// ── Ragdoll ────────────────────────────────────────────────────────────────────
//
// Demonstrates:
//   • RagdollDescriptor with full 15-bone humanoid
//   • PhysicsWorld::createRagdoll() producing articulated capsule bodies
//   • Dropping multiple ragdolls from height with randomised positions
//   • Mapping each bone body transform to a GLTF sphere node each frame
//   • A static floor to land on
//
// Controls:
//   Space — drop another ragdoll from above
//   Left-drag to orbit, right-drag / shift+drag to pan, scroll to zoom

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
#include <campello_physics/ragdoll.h>

#include <simd/simd.h>
#include <vector>
#include <memory>
#include <random>

namespace gpu  = systems::leal::campello_gpu;
namespace gltf = systems::leal::gltf;
namespace rend = systems::leal::campello_renderer;
namespace phys = campello::physics;
namespace vm   = systems::leal::vector_math;

// One ragdoll uses RagdollBone::Count = 15 GLTF nodes.
// GLBScene allocates kMaxBodies = 64 nodes (nodes 1..64, node 0 = floor).
// We can fit 4 full ragdolls (4 * 15 = 60 bones).
static constexpr int kBonesPerRagdoll = phys::RagdollBone::Count;  // 15
static constexpr int kMaxRagdolls     = 4;

@implementation ViewController {
    MTKView*  _metalView;
    std::shared_ptr<gpu::Device>    _device;
    std::shared_ptr<rend::Renderer> _renderer;
    std::shared_ptr<gltf::GLTF>     _gltf;
    Camera  _camera;
    NSPoint _lastMouse;
    BOOL    _mouseDown;
    BOOL    _rightMouseDown;

    phys::PhysicsWorld              _world;
    std::vector<phys::RagdollBody>  _ragdolls;
    int                             _nextNodeSlot;  // next free GLTF body node index

    std::default_random_engine _rng;
}

- (void)loadView {
    id<MTLDevice> mtlDev = MTLCreateSystemDefaultDevice();
    _metalView = [[MTKView alloc] initWithFrame:NSMakeRect(0, 0, 1280, 720) device:mtlDev];
    _metalView.colorPixelFormat         = MTLPixelFormatBGRA8Unorm;
    _metalView.depthStencilPixelFormat  = MTLPixelFormatInvalid;
    _metalView.clearColor               = MTLClearColorMake(0.07, 0.07, 0.11, 1.0);
    _metalView.preferredFramesPerSecond = 60;
    _metalView.delegate                 = self;
    self.view = _metalView;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [self setupRenderer];
    [self setupPhysics];
    _camera.radius = 18.f;
    _camera.theta  = 0.5f;
    _camera.target = { 0.f, 3.f, 0.f };
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
    _nextNodeSlot = 0;
    _rng.seed(1337);

    // Static floor
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(5.f, 0.5f, 5.f));
        d.transform.position = {0.f, -0.5f, 0.f};
        _world.createBody(d);
    }

    // Drop the first ragdoll immediately
    [self spawnRagdoll:{0.f, 6.f, 0.f}];
}

- (void)spawnRagdoll:(vm::Vector3<float>)rootPos {
    if ((int)_ragdolls.size() >= kMaxRagdolls) return;
    if (_nextNodeSlot + kBonesPerRagdoll > GLBScene::kMaxBodies) return;

    phys::RagdollDescriptor rd;
    rd.rootBodyType         = phys::BodyType::Dynamic;
    rd.totalMass            = 80.f;
    rd.rootTransform        = phys::Transform::identity();
    rd.rootTransform.position = rootPos;

    phys::RagdollBody ragdoll = _world.createRagdoll(rd);
    if (!ragdoll.isValid()) return;

    // Activate one GLTF sphere node per bone
    int firstSlot = _nextNodeSlot;
    for (int b = 0; b < kBonesPerRagdoll; ++b) {
        int nodeIdx = 1 + firstSlot + b;
        if ((size_t)nodeIdx >= _gltf->nodes->size()) break;
        auto& node = (*_gltf->nodes)[nodeIdx];
        node.scale = { 0.25, 0.25, 0.25 };  // small sphere representing each bone
        node.mesh  = 0;
    }
    _nextNodeSlot += kBonesPerRagdoll;

    _ragdolls.push_back(std::move(ragdoll));
}

- (void)updatePhysics:(double)dt {
    _world.step((float)dt);

    // Sync each ragdoll bone → GLTF node
    int slotBase = 0;
    for (const auto& ragdoll : _ragdolls) {
        for (int b = 0; b < kBonesPerRagdoll; ++b) {
            phys::Body bone = ragdoll.bone(b);
            if (!bone.isValid()) { continue; }

            int nodeIdx = 1 + slotBase + b;
            if ((size_t)nodeIdx >= _gltf->nodes->size()) break;

            const auto& bd = _world.bodyPool().get(bone.id());
            auto& node = (*_gltf->nodes)[nodeIdx];
            const auto& p = bd.transform.position;
            const auto& q = bd.transform.rotation;
            node.translation = { p.x(), p.y(), p.z() };
            node.rotation    = { (double)q.data[0], (double)q.data[1],
                                 (double)q.data[2], (double)q.data[3] };
        }
        slotBase += kBonesPerRagdoll;
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
    if (e.keyCode == 49) {  // Space
        std::uniform_real_distribution<float> dist(-1.5f, 1.5f);
        vm::Vector3<float> pos = { dist(_rng), 7.f, dist(_rng) };
        [self spawnRagdoll:pos];
    }
}

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)becomeFirstResponder  { return YES; }

@end
