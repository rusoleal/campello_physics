// ── Vehicle ────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   • VehicleDescriptor with 4-wheel suspension
//   • Throttle, braking, and steering via WASD / arrow keys
//   • A ramp obstacle to drive over
//   • Wheel position visualization via GLTF nodes
//   • Camera following the chassis
//
// Controls:
//   W / Up Arrow    — throttle
//   S / Down Arrow  — brake / reverse
//   A / Left Arrow  — steer left
//   D / Right Arrow — steer right
//   Left-drag to orbit camera, scroll to zoom
//
// Physics notes:
//   Chassis starts at y=1.0. Attachment points are at local y=-0.25 (world y=0.75).
//   restLength=0.45 + radius=0.35 → ray reaches y=−0.05, ensuring floor contact
//   from the very first frame.  Suspension is tuned to critical damping
//   (ζ≈1): stiffness=50000 N/m, damping=12000 N·s/m for 250 kg per wheel.
//   Chassis: linearDamping=0.2, angularDamping=4.0 to quench roll/pitch oscillation.
//   Floor: 100×100 m so the vehicle can drive without falling off.

#import "ViewController.h"
#import "Camera.hpp"
#import "GLBScene.h"

#import <campello_gpu/device.hpp>
#import <campello_gpu/texture_view.hpp>
#import <campello_gpu/constants/pixel_format.hpp>
#import <campello_renderer/campello_renderer.hpp>
#import <gltf/gltf.hpp>

#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/vehicle.h>

#include <simd/simd.h>
#include <cmath>
#include <vector>
#include <memory>
#include <unordered_set>

namespace gpu  = systems::leal::campello_gpu;
namespace gltf = systems::leal::gltf;
namespace rend = systems::leal::campello_renderer;
namespace phys = campello::physics;
namespace vm   = systems::leal::vector_math;

// GLTF node layout (1-based; node 0 = floor):
//   node 1  = chassis body
//   nodes 2-5 = 4 wheel spheres
//   node 6  = ramp box
static constexpr int   kChassisNodeIdx = 1;
static constexpr int   kWheelNodeBase  = 2;
static constexpr int   kRampNodeIdx    = 6;
static constexpr float kWheelRadius    = 0.35f;

// Local X and Z attachment positions for the 4 wheels
// (front-left, front-right, rear-left, rear-right)
static const float kWheelAttachX[4] = { -0.85f,  0.85f, -0.85f,  0.85f };
static const float kWheelAttachZ[4] = {  1.4f,   1.4f,  -1.4f,  -1.4f  };
static const float kWheelAttachY    = -0.25f;   // chassis local Y of attachment

@implementation ViewController {
    MTKView*  _metalView;
    std::shared_ptr<gpu::Device>    _device;
    std::shared_ptr<rend::Renderer> _renderer;
    std::shared_ptr<gltf::GLTF>     _gltf;
    Camera  _camera;
    NSPoint _lastMouse;
    BOOL    _mouseDown;
    BOOL    _rightMouseDown;

    phys::PhysicsWorld  _world;
    phys::VehicleBody   _vehicle;
    phys::Body          _ramp;

    std::unordered_set<unsigned short> _heldKeys;
}

- (void)loadView {
    id<MTLDevice> mtlDev = MTLCreateSystemDefaultDevice();
    _metalView = [[MTKView alloc] initWithFrame:NSMakeRect(0, 0, 1280, 720) device:mtlDev];
    _metalView.colorPixelFormat         = MTLPixelFormatBGRA8Unorm;
    _metalView.depthStencilPixelFormat  = MTLPixelFormatInvalid;
    _metalView.clearColor               = MTLClearColorMake(0.05, 0.08, 0.10, 1.0);
    _metalView.preferredFramesPerSecond = 60;
    _metalView.delegate                 = self;
    self.view = _metalView;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [self setupRenderer];
    [self setupPhysics];
    _camera.radius = 20.f;
    _camera.theta  = 0.4f;
}

- (void)setupRenderer {
    _device   = gpu::Device::createDefaultDevice(nullptr);
    _renderer = std::make_shared<rend::Renderer>(_device);
    _renderer->createDefaultPipelines(gpu::PixelFormat::bgra8unorm);
    auto glbData = GLBScene::build();
    _gltf = gltf::GLTF::loadGLB(glbData.data(), (uint64_t)glbData.size());
    _renderer->setAsset(_gltf);
    CGSize sz = _metalView.drawableSize;
    _renderer->resize((uint32_t)sz.width, (uint32_t)sz.height);
}

- (void)setupPhysics {
    _world.setGravity({0.f, -9.81f, 0.f});
    _world.setSubsteps(4);

    // Floor — 100×1×100 box, surface at y=0
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(50.f, 0.5f, 50.f));
        d.transform.position = {0.f, -0.5f, 0.f};
        (void)_world.createBody(d);
        // Scale the floor GLTF node to match the 100×100m physics body
        if (_gltf->nodes && !_gltf->nodes->empty()) {
            auto& floorNode = (*_gltf->nodes)[0];
            floorNode.scale = { 10.0, 1.0, 10.0 };
        }
    }

    // Ramp — gentle 12° incline ahead of the start position
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(1.5f, 0.15f, 3.f));
        d.transform.position = {0.f, 0.35f, -5.f};
        float angle = 0.21f; // ~12 degrees in radians
        d.transform.rotation = vm::Quaternion<float>(
            std::cos(angle * 0.5f), std::sin(angle * 0.5f), 0.f, 0.f);
        _ramp = _world.createBody(d);
        if ((size_t)kRampNodeIdx < _gltf->nodes->size()) {
            auto& node = (*_gltf->nodes)[kRampNodeIdx];
            node.scale = { 3.f, 0.3f, 6.f };
            node.mesh  = 1;
        }
    }

    // ── Vehicle ───────────────────────────────────────────────────────────────
    //
    // Chassis: 1000 kg box, starts at y=1.0.
    //
    // Suspension geometry:
    //   attachment local y = −0.25  →  world y = 0.75
    //   restLength = 0.45, radius = 0.35
    //   ray reaches y = 0.75 − 0.45 − 0.35 = −0.05  ← below floor → immediate contact
    //
    // Critical damping per wheel (250 kg each):
    //   ω₀ = √(50000/250) ≈ 14.14 rad/s
    //   ζ  = 7000 / (2 × 250 × 14.14) ≈ 1.0
    phys::VehicleDescriptor vd;
    {
        phys::BodyDescriptor& cd = vd.chassis;
        cd.type           = phys::BodyType::Dynamic;
        cd.shape          = std::make_shared<phys::BoxShape>(vm::Vector3<float>(0.9f, 0.25f, 1.8f));
        cd.mass           = 1000.f;
        cd.transform.position = {0.f, 1.f, 0.f};
        cd.linearDamping  = 0.2f;
        cd.angularDamping = 4.0f;
    }
    vd.maxEngineForce = 3000.f;
    vd.maxBrakeForce  = 8000.f;

    float maxSteer = 0.45f;
    for (int i = 0; i < 4; ++i) {
        phys::WheelDescriptor wd;
        wd.attachmentLocal  = { kWheelAttachX[i], kWheelAttachY, kWheelAttachZ[i] };
        wd.restLength       = 0.45f;   // long enough for wheels to reach floor from y=1.0
        wd.radius           = kWheelRadius;
        wd.stiffness        = 50000.f; // N/m
        wd.damping          = 12000.f; // N·s/m — accounts for discrete-time lag at 60 Hz
        wd.lateralFriction  =  1.2f;
        wd.maxSteerAngle    = (i < 2) ? maxSteer : 0.f;
        vd.wheels.push_back(wd);
    }

    _vehicle = _world.createVehicle(vd);

    // Activate chassis GLTF node
    if ((size_t)kChassisNodeIdx < _gltf->nodes->size()) {
        auto& node = (*_gltf->nodes)[kChassisNodeIdx];
        node.scale = { 1.8f, 0.5f, 3.6f };
        node.mesh  = 1;
    }
    // Activate 4 wheel nodes — sphere mesh scaled to wheel diameter
    float diam = kWheelRadius * 2.f;
    for (int i = 0; i < 4; ++i) {
        int idx = kWheelNodeBase + i;
        if ((size_t)idx < _gltf->nodes->size()) {
            auto& node = (*_gltf->nodes)[idx];
            node.scale = { diam, diam, diam };
            node.mesh  = 0;
        }
    }
}

// Rotate a vector by a quaternion stored as (x,y,z,w) in q.data[0..3].
static vm::Vector3<float> rotateByQuat(const vm::Vector3<float>& v,
                                       const vm::Quaternion<float>& q)
{
    float qx = q.data[0], qy = q.data[1], qz = q.data[2], qw = q.data[3];
    float lx = v.x(), ly = v.y(), lz = v.z();
    // t = 2 * (q.xyz × v)
    float tx = 2.f * (qy * lz - qz * ly);
    float ty = 2.f * (qz * lx - qx * lz);
    float tz = 2.f * (qx * ly - qy * lx);
    // result = v + qw * t + q.xyz × t
    return {
        lx + qw * tx + qy * tz - qz * ty,
        ly + qw * ty + qz * tx - qx * tz,
        lz + qw * tz + qx * ty - qy * tx
    };
}

- (void)updatePhysics:(double)dt {
    // Map held keys → vehicle controls
    bool fwd   = _heldKeys.count(13) || _heldKeys.count(126); // W / ↑
    bool back  = _heldKeys.count(1)  || _heldKeys.count(125); // S / ↓
    bool left  = _heldKeys.count(0)  || _heldKeys.count(123); // A / ←
    bool right = _heldKeys.count(2)  || _heldKeys.count(124); // D / →

    _vehicle.throttle = fwd  ? 1.f : 0.f;
    _vehicle.braking  = back ? 0.8f : 0.f;
    _vehicle.steering = left ? -1.f : (right ? 1.f : 0.f);

    _world.syncVehicleControls(_vehicle);
    _world.step((float)dt);

    // ── Chassis → GLTF node ───────────────────────────────────────────────────
    phys::Body chassis = _vehicle.chassisBody();
    const auto& cd = _world.bodyPool().get(chassis.id());
    const auto& cp = cd.transform.position;
    const auto& cq = cd.transform.rotation;
    {
        auto& node = (*_gltf->nodes)[kChassisNodeIdx];
        node.translation = { cp.x(), cp.y(), cp.z() };
        node.rotation    = { (double)cq.data[0], (double)cq.data[1],
                             (double)cq.data[2], (double)cq.data[3] };
    }

    // ── Wheels → GLTF nodes ───────────────────────────────────────────────────
    // Always derive wheel centre from chassis transform + current suspension length.
    // suspensionLength == restLength when airborne, so no mode-switch → no blinking.
    int wCount = _world.vehicleWheelCount(_vehicle);
    for (int i = 0; i < wCount && i < 4; ++i) {
        const phys::WheelState& ws = _world.vehicleWheelState(_vehicle, i);

        // Wheel centre = attachment_world + downWorld * (suspensionLength + radius)
        vm::Vector3<float> downLocal(0.f, -1.f, 0.f);
        vm::Vector3<float> downWorld = rotateByQuat(downLocal, cq);
        vm::Vector3<float> attachLocal = { kWheelAttachX[i], kWheelAttachY, kWheelAttachZ[i] };
        vm::Vector3<float> attachWorld = cp + rotateByQuat(attachLocal, cq);
        float offset = ws.suspensionLength;  // attachment → wheel centre (radius already subtracted in physics)
        float wx = attachWorld.x() + downWorld.x() * offset;
        float wy = attachWorld.y() + downWorld.y() * offset;
        float wz = attachWorld.z() + downWorld.z() * offset;

        int nodeIdx = kWheelNodeBase + i;
        if ((size_t)nodeIdx < _gltf->nodes->size()) {
            auto& node = (*_gltf->nodes)[nodeIdx];
            node.translation = { wx, wy, wz };
            node.rotation    = { (double)cq.data[0], (double)cq.data[1],
                                 (double)cq.data[2], (double)cq.data[3] };
        }
    }

    // ── Ramp → GLTF node (static, just sync once) ────────────────────────────
    {
        const auto& rd = _world.bodyPool().get(_ramp.id());
        if ((size_t)kRampNodeIdx < _gltf->nodes->size()) {
            auto& node = (*_gltf->nodes)[kRampNodeIdx];
            const auto& p = rd.transform.position;
            const auto& q = rd.transform.rotation;
            node.translation = { p.x(), p.y(), p.z() };
            node.rotation    = { (double)q.data[0], (double)q.data[1],
                                 (double)q.data[2], (double)q.data[3] };
        }
    }

    // Camera follows chassis
    _camera.target = { (float)cp.x(), (float)cp.y(), (float)cp.z() };
}

- (void)drawInMTKView:(MTKView*)view {
    if (!_renderer) return;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!drawable || !drawable.texture) return;
    [self updatePhysics:1.0/60.0];
    CGSize sz = view.drawableSize;
    if (sz.width < 1 || sz.height < 1) return;
    float aspect = (float)(sz.width / sz.height);
    simd_float4x4 vm2 = _camera.viewMatrix();
    simd_float4x4 pm  = _camera.projectionMatrix(aspect);
    _renderer->setCameraMatrices((const float*)&vm2, (const float*)&pm);
    auto cv = gpu::TextureView::fromNative((__bridge void*)drawable.texture);
    if (cv) _renderer->render(cv);
    [drawable present];
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
    if (_renderer) _renderer->resize((uint32_t)size.width, (uint32_t)size.height);
}

- (void)mouseDown:(NSEvent*)e      { _lastMouse=[e locationInWindow]; _mouseDown=YES; }
- (void)mouseUp:(NSEvent*)e        { _mouseDown=NO; }
- (void)rightMouseDown:(NSEvent*)e { _lastMouse=[e locationInWindow]; _rightMouseDown=YES; }
- (void)rightMouseUp:(NSEvent*)e   { _rightMouseDown=NO; }

- (void)mouseDragged:(NSEvent*)e {
    if (!_mouseDown) return;
    NSPoint c=[e locationInWindow];
    float dx=(float)(c.x-_lastMouse.x), dy=(float)(c.y-_lastMouse.y);
    _lastMouse=c;
    if (e.modifierFlags & NSEventModifierFlagShift) _camera.pan(dx,dy);
    else _camera.orbit(-dx*0.005f, dy*0.005f);
}
- (void)rightMouseDragged:(NSEvent*)e {
    if (!_rightMouseDown) return;
    NSPoint c=[e locationInWindow];
    float dx=(float)(c.x-_lastMouse.x), dy=(float)(c.y-_lastMouse.y);
    _lastMouse=c; _camera.pan(dx,dy);
}
- (void)scrollWheel:(NSEvent*)e {
    _camera.zoom((float)e.deltaY*(e.hasPreciseScrollingDeltas?0.2f:1.0f));
}

- (void)keyDown:(NSEvent*)e { _heldKeys.insert(e.keyCode); }
- (void)keyUp:(NSEvent*)e   { _heldKeys.erase(e.keyCode); }

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)becomeFirstResponder  { return YES; }

@end
