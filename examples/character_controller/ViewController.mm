// ── Character Controller ───────────────────────────────────────────────────────
//
// Demonstrates:
//   • Kinematic capsule character moving on a floor using forces/velocities
//   • WASD movement, jump with Space
//   • Raycast-based ground detection
//   • Obstacles (static boxes) to navigate around
//
// The character is a kinematic capsule. Its horizontal velocity is set directly
// (kinematic control); vertical velocity is accumulated via gravity + jump impulse.
// A downward raycast determines whether the character is grounded.

#import "ViewController.h"
#import "Camera.hpp"
#import "GLBScene.h"

#import <campello_gpu/device.hpp>
#import <campello_gpu/texture_view.hpp>
#import <campello_gpu/constants/pixel_format.hpp>
#import <campello_renderer/campello_renderer.hpp>
#import <gltf/gltf.hpp>

#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/query.h>

#include <simd/simd.h>
#include <vector>
#include <memory>
#include <unordered_set>

namespace gpu  = systems::leal::campello_gpu;
namespace gltf = systems::leal::gltf;
namespace rend = systems::leal::campello_renderer;
namespace phys = campello::physics;
namespace vm   = systems::leal::vector_math;

@implementation ViewController {
    MTKView*  _metalView;
    std::shared_ptr<gpu::Device>    _device;
    std::shared_ptr<rend::Renderer> _renderer;
    std::shared_ptr<gltf::GLTF>     _gltf;
    Camera  _camera;
    NSPoint _lastMouse;
    BOOL    _mouseDown;
    BOOL    _rightMouseDown;

    // Physics
    phys::PhysicsWorld _world;
    phys::Body         _character;  // dynamic sphere representing the character
    float              _verticalVelocity;
    BOOL               _grounded;

    std::vector<phys::Body> _obstacles;  // static boxes
    int                     _nextSlot;

    // Held keys
    std::unordered_set<unsigned short> _heldKeys;
}

- (void)loadView {
    id<MTLDevice> mtlDev = MTLCreateSystemDefaultDevice();
    _metalView = [[MTKView alloc] initWithFrame:NSMakeRect(0, 0, 1280, 720) device:mtlDev];
    _metalView.colorPixelFormat         = MTLPixelFormatBGRA8Unorm;
    _metalView.depthStencilPixelFormat  = MTLPixelFormatInvalid;
    _metalView.clearColor               = MTLClearColorMake(0.06, 0.09, 0.12, 1.0);
    _metalView.preferredFramesPerSecond = 60;
    _metalView.delegate                 = self;
    self.view = _metalView;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [self setupRenderer];
    [self setupPhysics];
    _camera.radius = 15.f;
    _camera.theta  = 0.5f;
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
    _nextSlot = 0;

    // Floor
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(5.f, 0.5f, 5.f));
        d.transform.position = {0.f, -0.5f, 0.f};
        (void)_world.createBody(d);
    }

    // Character sphere (radius 0.5)
    {
        phys::BodyDescriptor d;
        d.type   = phys::BodyType::Dynamic;
        d.shape  = std::make_shared<phys::SphereShape>(0.5f);
        d.mass   = 70.f;
        d.linearDamping  = 5.f;   // high damping for direct velocity control
        d.angularDamping = 1.f;
        d.transform.position = {0.f, 1.f, 0.f};
        _character = _world.createBody(d);
        [self activateNode:0];
    }

    // Obstacle boxes
    float ox[4] = {2.f, -2.f, 3.f, -3.f};
    float oz[4] = {2.f,  2.f,-2.f,  0.f};
    for (int i = 0; i < 4; ++i) {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(0.5f, 0.5f, 0.5f));
        d.transform.position = {ox[i], 0.5f, oz[i]};
        _obstacles.push_back(_world.createBody(d));
        [self activateNode:i + 1 mesh:GLBScene::MeshType::Box];
    }
}

- (void)activateNode:(int)slot {
    [self activateNode:slot mesh:GLBScene::MeshType::Sphere];
}

- (void)activateNode:(int)slot mesh:(GLBScene::MeshType)type {
    int nodeIdx = 1 + slot;
    if ((size_t)nodeIdx >= _gltf->nodes->size()) return;
    auto& node = (*_gltf->nodes)[nodeIdx];
    node.scale = { 1.0, 1.0, 1.0 };
    // Switch to box mesh if needed (mesh index 1)
    if (type == GLBScene::MeshType::Box) node.mesh = 1;
}

- (void)updatePhysics:(double)dt {
    // Read WASD movement from held keys
    const float speed = 5.f;
    vm::Vector3<float> move = {0, 0, 0};
    if (_heldKeys.count(13)) move = move + vm::Vector3<float>(0, 0, -1); // W
    if (_heldKeys.count(1))  move = move + vm::Vector3<float>(0, 0,  1); // S
    if (_heldKeys.count(0))  move = move + vm::Vector3<float>(-1, 0, 0); // A
    if (_heldKeys.count(2))  move = move + vm::Vector3<float>( 1, 0, 0); // D

    float len = sqrtf(move.x()*move.x() + move.z()*move.z());
    if (len > 0.01f) {
        move = vm::Vector3<float>(move.x()/len * speed, 0, move.z()/len * speed);
        _character.applyForce(move * 70.f);  // force proportional to mass
    }

    // Ground check via downward raycast
    phys::Ray ray;
    const auto& bd = _world.bodyPool().get(_character.id());
    ray.origin    = bd.transform.position;
    ray.direction = {0.f, -1.f, 0.f};
    ray.maxDistance = 0.65f;  // slightly more than sphere radius
    auto hit = _world.raycastClosest(ray);
    _grounded = hit.has_value();

    // Gravity + jump are now handled by the physics engine
    _world.step((float)dt);

    // Update camera target to follow character
    _camera.target = {
        (float)bd.transform.position.x(),
        (float)bd.transform.position.y(),
        (float)bd.transform.position.z()
    };

    // Sync transforms → GLTF nodes
    // Character (slot 0)
    {
        auto& node = (*_gltf->nodes)[1];
        const auto& p = bd.transform.position;
        node.translation = { p.x(), p.y(), p.z() };
    }
    // Obstacles (slots 1-4) - static, set once but keep in sync
    for (size_t i = 0; i < _obstacles.size(); ++i) {
        int nodeIdx = 2 + (int)i;
        if ((size_t)nodeIdx >= _gltf->nodes->size()) break;
        const auto& ob = _world.bodyPool().get(_obstacles[i].id());
        auto& node = (*_gltf->nodes)[nodeIdx];
        const auto& p = ob.transform.position;
        node.translation = { p.x(), p.y(), p.z() };
    }
}

- (void)drawInMTKView:(MTKView*)view {
    if (!_renderer) return;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!drawable || !drawable.texture) return;
    [self updatePhysics:1.0/60.0];
    CGSize sz = view.drawableSize;
    if (sz.width < 1 || sz.height < 1) return;
    float aspect = (float)(sz.width / sz.height);
    simd_float4x4 vm = _camera.viewMatrix();
    simd_float4x4 pm = _camera.projectionMatrix(aspect);
    _renderer->setCameraMatrices((const float*)&vm, (const float*)&pm);
    auto cv = gpu::TextureView::fromNative((__bridge void*)drawable.texture);
    if (cv) _renderer->render(cv);
    [drawable present];
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
    if (_renderer) _renderer->resize((uint32_t)size.width, (uint32_t)size.height);
}

- (void)mouseDown:(NSEvent*)e   { _lastMouse=[e locationInWindow]; _mouseDown=YES; }
- (void)mouseUp:(NSEvent*)e     { _mouseDown=NO; }
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

- (void)keyDown:(NSEvent*)e {
    _heldKeys.insert(e.keyCode);
    // Space: jump impulse
    if (e.keyCode == 49 && _grounded)
        _character.applyLinearImpulse({0.f, 350.f, 0.f});
}
- (void)keyUp:(NSEvent*)e { _heldKeys.erase(e.keyCode); }

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)becomeFirstResponder  { return YES; }

@end
