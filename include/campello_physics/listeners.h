#pragma once

#include <campello_physics/body.h>
#include <campello_physics/contact.h>

namespace campello::physics {

struct IStepListener {
    virtual void onPreStep(float /*dt*/)  {}
    virtual void onPostStep(float /*dt*/) {}
    virtual ~IStepListener() = default;
};

struct IContactListener {
    virtual void onContactAdded    (Body /*a*/, Body /*b*/, const ContactManifold& /*m*/) {}
    virtual void onContactPersisted(Body /*a*/, Body /*b*/, const ContactManifold& /*m*/) {}
    virtual void onContactRemoved  (Body /*a*/, Body /*b*/) {}
    virtual ~IContactListener() = default;
};

struct ITriggerListener {
    virtual void onTriggerEnter(Body /*sensor*/, Body /*other*/) {}
    virtual void onTriggerExit (Body /*sensor*/, Body /*other*/) {}
    virtual ~ITriggerListener() = default;
};

} // namespace campello::physics
