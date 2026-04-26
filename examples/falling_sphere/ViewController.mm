// ── Falling Sphere ─────────────────────────────────────────────────────────────
//
// Demonstrates:
//   • Creating a static floor and multiple dynamic spheres
//   • Gravity, contact response, and sleep/wake cycle
//   • Mapping physics body transforms to GLTF node transforms each frame
//   • Rendering via campello_renderer
//
// Controls: left-drag to orbit, right-drag / shift+drag to pan, scroll to zoom.
// Press Space to drop another sphere from a random position above the floor.

#import "ViewController.h"
#import "AppDelegate.h"
#import "Camera.hpp"
#import "GLBScene.h"

#import <campello_gpu/device.hpp>
#import <campello_gpu/texture_view.hpp>
#import <campello_gpu/constants/pixel_format.hpp>
#import <campello_renderer/campello_renderer.hpp>
#import <gltf/gltf.hpp>

#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>

#include <simd/simd.h>
#include <vector>
#include <memory>
#include <random>

namespace gpu  = systems::leal::campello_gpu;
namespace gltf = systems::leal::gltf;
namespace rend = systems::leal::campello_renderer;
namespace phys = campello::physics;
namespace vm   = systems::leal::vector_math;

// ── ViewController implementation ─────────────────────────────────────────────

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
    phys::PhysicsWorld              _world;
    std::vector<phys::Body>         _bodies;   // dynamic spheres
    phys::Body                      _floor;
    int                             _nextBodySlot;  // which GLTF node slot to use next

    std::default_random_engine      _rng;
}

// ── View lifecycle ─────────────────────────────────────────────────────────────

- (void)loadView {
    id<MTLDevice> mtlDev = MTLCreateSystemDefaultDevice();
    _metalView = [[MTKView alloc] initWithFrame:NSMakeRect(0, 0, 1280, 720) device:mtlDev];
    _metalView.colorPixelFormat         = MTLPixelFormatBGRA8Unorm;
    _metalView.depthStencilPixelFormat  = MTLPixelFormatInvalid; // renderer manages depth
    _metalView.clearColor               = MTLClearColorMake(0.08, 0.08, 0.12, 1.0);
    _metalView.preferredFramesPerSecond = 60;
    _metalView.delegate                 = self;
    self.view = _metalView;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [self setupRenderer];
    [self setupPhysics];
    [self setupCamera];
}

// ── Renderer setup ─────────────────────────────────────────────────────────────

- (void)setupRenderer {
    _device   = gpu::Device::createDefaultDevice(nullptr);
    _renderer = std::make_shared<rend::Renderer>(_device);
    _renderer->createDefaultPipelines(gpu::PixelFormat::bgra8unorm);

    // Build and load the procedural GLB scene (sphere, box, floor meshes)
    auto glbData = GLBScene::build();
    _gltf = gltf::GLTF::loadGLB(glbData.data(), (uint64_t)glbData.size());
    _renderer->setAsset(_gltf);

    CGSize sz = _metalView.drawableSize;
    _renderer->resize((uint32_t)sz.width, (uint32_t)sz.height);
}

// ── Physics setup ──────────────────────────────────────────────────────────────

- (void)setupPhysics {
    _world.setGravity({0.f, -9.81f, 0.f});
    _nextBodySlot = 0;
    _rng.seed(42);

    // Static floor — matches the 10×10 floor plane in the GLB scene
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(
            vm::Vector3<float>(5.f, 0.5f, 5.f));
        d.transform.position = {0.f, -0.5f, 0.f};
        _floor = _world.createBody(d);
    }

    // Spawn an initial stack of spheres
    [self spawnSphere:{0.f, 2.f, 0.f}];
    [self spawnSphere:{0.3f, 4.f, 0.1f}];
    [self spawnSphere:{-0.2f, 6.f, -0.1f}];
}

- (void)spawnSphere:(vm::Vector3<float>)position {
    if (_nextBodySlot >= GLBScene::kMaxBodies) return;

    phys::BodyDescriptor d;
    d.type   = phys::BodyType::Dynamic;
    d.shape  = std::make_shared<phys::SphereShape>(0.5f);
    d.mass   = 1.f;
    d.transform.position = position;
    _bodies.push_back(_world.createBody(d));

    // Activate the corresponding GLTF node (mesh = 0 = sphere, scale = 1)
    int nodeIdx = 1 + _nextBodySlot;  // node 0 = floor
    if ((size_t)nodeIdx < _gltf->nodes->size()) {
        auto& node = (*_gltf->nodes)[nodeIdx];
        node.scale = { 1.0, 1.0, 1.0 };
    }
    ++_nextBodySlot;
}

- (void)setupCamera {
    _camera.radius = 12.f;
    _camera.theta  = 0.4f;
    _camera.target = { 0.f, 2.f, 0.f };
}

// ── Per-frame update ──────────────────────────────────────────────────────────

- (void)updatePhysics:(double)dt {
    _world.step((float)dt);

    // Sync each sphere body → GLTF node transform
    for (size_t i = 0; i < _bodies.size(); ++i) {
        int nodeIdx = 1 + (int)i;
        if ((size_t)nodeIdx >= _gltf->nodes->size()) break;

        const auto& bd = _world.bodyPool().get(_bodies[i].id());
        auto& node = (*_gltf->nodes)[nodeIdx];

        const auto& p = bd.transform.position;
        const auto& q = bd.transform.rotation;

        node.translation = { (double)p.x(), (double)p.y(), (double)p.z() };
        node.rotation    = { (double)q.data[0], (double)q.data[1],
                             (double)q.data[2], (double)q.data[3] };
    }
}

// ── MTKViewDelegate ───────────────────────────────────────────────────────────

- (void)drawInMTKView:(MTKView*)view {
    if (!_renderer) return;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!drawable || !drawable.texture) return;

    const double dt = 1.0 / 60.0;
    [self updatePhysics:dt];

    CGSize sz = view.drawableSize;
    if (sz.width < 1 || sz.height < 1) return;

    float aspect = (float)(sz.width / sz.height);
    simd_float4x4 viewMat = _camera.viewMatrix();
    simd_float4x4 projMat = _camera.projectionMatrix(aspect);
    _renderer->setCameraMatrices((const float*)&viewMat, (const float*)&projMat);

    auto colorView = gpu::TextureView::fromNative((__bridge void*)drawable.texture);
    if (colorView) _renderer->render(colorView);
    [drawable present];
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
    if (_renderer) _renderer->resize((uint32_t)size.width, (uint32_t)size.height);
}

// ── Input ─────────────────────────────────────────────────────────────────────

- (void)mouseDown:(NSEvent*)e   { _lastMouse = [e locationInWindow]; _mouseDown = YES; }
- (void)mouseUp:(NSEvent*)e     { _mouseDown = NO; }
- (void)rightMouseDown:(NSEvent*)e { _lastMouse = [e locationInWindow]; _rightMouseDown = YES; }
- (void)rightMouseUp:(NSEvent*)e   { _rightMouseDown = NO; }

- (void)mouseDragged:(NSEvent*)e {
    if (!_mouseDown) return;
    NSPoint cur = [e locationInWindow];
    float dx = (float)(cur.x - _lastMouse.x);
    float dy = (float)(cur.y - _lastMouse.y);
    _lastMouse = cur;
    if (e.modifierFlags & NSEventModifierFlagShift) _camera.pan(dx, dy);
    else _camera.orbit(-dx * 0.005f, dy * 0.005f);
}

- (void)rightMouseDragged:(NSEvent*)e {
    if (!_rightMouseDown) return;
    NSPoint cur = [e locationInWindow];
    float dx = (float)(cur.x - _lastMouse.x);
    float dy = (float)(cur.y - _lastMouse.y);
    _lastMouse = cur;
    _camera.pan(dx, dy);
}

- (void)scrollWheel:(NSEvent*)e {
    _camera.zoom((float)e.deltaY * (e.hasPreciseScrollingDeltas ? 0.2f : 1.0f));
}

- (void)keyDown:(NSEvent*)e {
    if ([e.charactersIgnoringModifiers isEqualToString:@" "]) {
        std::uniform_real_distribution<float> dist(-2.f, 2.f);
        vm::Vector3<float> pos = { dist(_rng), 8.f, dist(_rng) };
        [self spawnSphere:pos];
    }
}

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)becomeFirstResponder  { return YES; }

@end
