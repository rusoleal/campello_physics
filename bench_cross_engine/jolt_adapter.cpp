#include "jolt_adapter.h"
#include <Jolt/Physics/Body/BodyInterface.h>
#include <cmath>

static bool s_joltInitialized = false;

JoltAdapter::JoltAdapter() {
    if (!s_joltInitialized) {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
        s_joltInitialized = true;
    }
}

JoltAdapter::~JoltAdapter() {
    // We intentionally do NOT call UnregisterTypes / delete Factory here
    // because multiple adapters may exist across benchmark iterations.
}

void JoltAdapter::reset() {
    m_physicsSystem.reset();
    m_jobSystem.reset();
    m_tempAllocator.reset();
    m_broadPhaseLayerInterface.reset();
    m_objectVsBroadPhaseLayerFilter.reset();
    m_objectLayerPairFilter.reset();
}

void JoltAdapter::init(int workerThreads) {
    m_workerThreads = workerThreads;
    reset();
}

void JoltAdapter::setupFreeFall(int bodyCount) {
    reset();

    m_tempAllocator = std::make_unique<TempAllocatorImpl>(100 * 1024 * 1024);
    m_jobSystem = std::make_unique<JobSystemThreadPool>(
        cMaxPhysicsJobs, cMaxPhysicsBarriers,
        m_workerThreads > 1 ? static_cast<uint>(m_workerThreads) : 0);

    m_broadPhaseLayerInterface = std::make_unique<BPLInterface>();
    m_objectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    m_objectLayerPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

    const uint maxBodies = static_cast<uint>(bodyCount + 1024);
    const uint maxBodyPairs = maxBodies * 10;
    const uint maxContactConstraints = maxBodies * 10;

    m_physicsSystem = std::make_unique<PhysicsSystem>();
    m_physicsSystem->Init(
        maxBodies, 0, maxBodyPairs, maxContactConstraints,
        *m_broadPhaseLayerInterface,
        *m_objectVsBroadPhaseLayerFilter,
        *m_objectLayerPairFilter);

    m_physicsSystem->SetGravity(Vec3(0.f, -9.81f, 0.f));

    BodyInterface& bi = m_physicsSystem->GetBodyInterface();

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 2.5f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        Vec3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.1f,
            (float(iz) - side * 0.5f) * spacing);

        BodyCreationSettings settings(
            new SphereShape(0.5f), pos, Quat::sIdentity(),
            EMotionType::Dynamic, Layers::MOVING);
        settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = 1.0f;
        Body* body = bi.CreateBody(settings);
        body->SetAllowSleeping(false);
        bi.AddBody(body->GetID(), EActivation::Activate);
    }
}

void JoltAdapter::setupPile(int bodyCount) {
    reset();

    m_tempAllocator = std::make_unique<TempAllocatorImpl>(100 * 1024 * 1024);
    m_jobSystem = std::make_unique<JobSystemThreadPool>(
        cMaxPhysicsJobs, cMaxPhysicsBarriers,
        m_workerThreads > 1 ? static_cast<uint>(m_workerThreads) : 0);

    m_broadPhaseLayerInterface = std::make_unique<BPLInterface>();
    m_objectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    m_objectLayerPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

    const uint maxBodies = static_cast<uint>(bodyCount + 1024);
    const uint maxBodyPairs = maxBodies * 10;
    const uint maxContactConstraints = maxBodies * 10;

    m_physicsSystem = std::make_unique<PhysicsSystem>();
    m_physicsSystem->Init(
        maxBodies, 0, maxBodyPairs, maxContactConstraints,
        *m_broadPhaseLayerInterface,
        *m_objectVsBroadPhaseLayerFilter,
        *m_objectLayerPairFilter);

    m_physicsSystem->SetGravity(Vec3(0.f, -9.81f, 0.f));

    BodyInterface& bi = m_physicsSystem->GetBodyInterface();

    // Floor
    BodyCreationSettings floorSettings(
        new BoxShape(Vec3(200.f, 0.5f, 200.f)),
        Vec3(0.f, -0.5f, 0.f), Quat::sIdentity(),
        EMotionType::Static, Layers::NON_MOVING);
    Body* floor = bi.CreateBody(floorSettings);
    bi.AddBody(floor->GetID(), EActivation::DontActivate);

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        Vec3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);

        BodyCreationSettings settings(
            new BoxShape(Vec3(0.5f, 0.5f, 0.5f)), pos, Quat::sIdentity(),
            EMotionType::Dynamic, Layers::MOVING);
        settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = 1.0f;
        Body* body = bi.CreateBody(settings);
        body->SetAllowSleeping(false);
        bi.AddBody(body->GetID(), EActivation::Activate);
    }
}

void JoltAdapter::setupConstraintChains(int chains, int linksPerChain) {
    reset();

    m_tempAllocator = std::make_unique<TempAllocatorImpl>(100 * 1024 * 1024);
    m_jobSystem = std::make_unique<JobSystemThreadPool>(
        cMaxPhysicsJobs, cMaxPhysicsBarriers,
        m_workerThreads > 1 ? static_cast<uint>(m_workerThreads) : 0);

    m_broadPhaseLayerInterface = std::make_unique<BPLInterface>();
    m_objectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    m_objectLayerPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

    const int totalBodies = chains * linksPerChain + chains + 1024;
    const uint maxBodies = static_cast<uint>(totalBodies);
    const uint maxBodyPairs = maxBodies * 10;
    const uint maxContactConstraints = maxBodies * 10;

    m_physicsSystem = std::make_unique<PhysicsSystem>();
    m_physicsSystem->Init(
        maxBodies, 0, maxBodyPairs, maxContactConstraints,
        *m_broadPhaseLayerInterface,
        *m_objectVsBroadPhaseLayerFilter,
        *m_objectLayerPairFilter);

    m_physicsSystem->SetGravity(Vec3(0.f, -9.81f, 0.f));

    BodyInterface& bi = m_physicsSystem->GetBodyInterface();

    // Floor
    BodyCreationSettings floorSettings(
        new BoxShape(Vec3(200.f, 0.5f, 200.f)),
        Vec3(0.f, -0.5f, 0.f), Quat::sIdentity(),
        EMotionType::Static, Layers::NON_MOVING);
    Body* floor = bi.CreateBody(floorSettings);
    bi.AddBody(floor->GetID(), EActivation::DontActivate);

    const float linkHalfH = 0.3f;
    const float linkSpacing = linkHalfH * 2.f + 0.05f;

    for (int c = 0; c < chains; ++c) {
        Body* prev = nullptr;
        for (int l = 0; l < linksPerChain; ++l) {
            Vec3 pos(
                float(c) * 2.f,
                5.f + float(l) * linkSpacing,
                0.f);

            BodyCreationSettings settings(
                new CapsuleShape(linkHalfH, 0.15f), pos, Quat::sIdentity(),
                EMotionType::Dynamic, Layers::MOVING);
            settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = 1.0f;
            Body* body = bi.CreateBody(settings);
            bi.AddBody(body->GetID(), EActivation::Activate);

            if (l == 0) {
                // Anchor to static world anchor
                Vec3 anchorPos(float(c) * 2.f, 5.f - linkSpacing, 0.f);
                BodyCreationSettings anchorSettings(
                    new SphereShape(0.1f), anchorPos, Quat::sIdentity(),
                    EMotionType::Static, Layers::NON_MOVING);
                Body* anchor = bi.CreateBody(anchorSettings);
                bi.AddBody(anchor->GetID(), EActivation::DontActivate);

                PointConstraintSettings constraint;
                constraint.mPoint1 = constraint.mPoint2 =
                    Vec3(float(c) * 2.f, 5.f - linkSpacing * 0.5f, 0.f);
                m_physicsSystem->AddConstraint(
                    constraint.Create(*anchor, *body));
            } else {
                PointConstraintSettings constraint;
                constraint.mPoint1 = constraint.mPoint2 =
                    Vec3(float(c) * 2.f, 5.f + (float(l) - 0.5f) * linkSpacing, 0.f);
                m_physicsSystem->AddConstraint(
                    constraint.Create(*prev, *body));
            }
            prev = body;
        }
    }
}

void JoltAdapter::setupMixedScene(int bodyCount) {
    reset();

    m_tempAllocator = std::make_unique<TempAllocatorImpl>(100 * 1024 * 1024);
    m_jobSystem = std::make_unique<JobSystemThreadPool>(
        cMaxPhysicsJobs, cMaxPhysicsBarriers,
        m_workerThreads > 1 ? static_cast<uint>(m_workerThreads) : 0);

    m_broadPhaseLayerInterface = std::make_unique<BPLInterface>();
    m_objectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    m_objectLayerPairFilter = std::make_unique<ObjectLayerPairFilterImpl>();

    const uint maxBodies = static_cast<uint>(bodyCount + 1024);
    const uint maxBodyPairs = maxBodies * 10;
    const uint maxContactConstraints = maxBodies * 10;

    m_physicsSystem = std::make_unique<PhysicsSystem>();
    m_physicsSystem->Init(
        maxBodies, 0, maxBodyPairs, maxContactConstraints,
        *m_broadPhaseLayerInterface,
        *m_objectVsBroadPhaseLayerFilter,
        *m_objectLayerPairFilter);

    m_physicsSystem->SetGravity(Vec3(0.f, -9.81f, 0.f));

    BodyInterface& bi = m_physicsSystem->GetBodyInterface();

    // Floor
    BodyCreationSettings floorSettings(
        new BoxShape(Vec3(200.f, 0.5f, 200.f)),
        Vec3(0.f, -0.5f, 0.f), Quat::sIdentity(),
        EMotionType::Static, Layers::NON_MOVING);
    Body* floor = bi.CreateBody(floorSettings);
    bi.AddBody(floor->GetID(), EActivation::DontActivate);

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        Vec3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);

        Shape* shape = nullptr;
        if (i % 3 == 0) shape = new BoxShape(Vec3(0.5f, 0.5f, 0.5f));
        else if (i % 3 == 1) shape = new SphereShape(0.5f);
        else shape = new CapsuleShape(0.4f, 0.3f);

        BodyCreationSettings settings(shape, pos, Quat::sIdentity(),
            EMotionType::Dynamic, Layers::MOVING);
        settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = 1.0f;
        Body* body = bi.CreateBody(settings);
        body->SetAllowSleeping(false);
        bi.AddBody(body->GetID(), EActivation::Activate);
    }
}

void JoltAdapter::step(float dt) {
    m_physicsSystem->Update(dt, 1, m_tempAllocator.get(), m_jobSystem.get());
}
