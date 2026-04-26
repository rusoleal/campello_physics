#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/cylinder_shape.h>
#include <campello_physics/shapes/convex_hull_shape.h>
#include <campello_physics/shapes/compound_shape.h>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace campello::physics {

// ══════════════════════════════════════════════════════════════════════════════
// JSON writer helpers
// ══════════════════════════════════════════════════════════════════════════════

namespace {

struct Writer {
    std::string out;
    int         indent = 0;

    void raw(char c)                    { out += c; }
    void raw(std::string_view s)        { out.append(s.data(), s.size()); }

    void nl()  { raw('\n'); for (int i=0;i<indent;++i) raw("  "); }
    void comma_nl() { raw(','); nl(); }

    void key(std::string_view k) {
        raw('"'); out.append(k.data(), k.size()); raw('"'); raw(':'); raw(' ');
    }
    void str(std::string_view v) { raw('"'); out.append(v.data(), v.size()); raw('"'); }

    void num(float v) {
        char buf[32];
        // Use %.8g to avoid unnecessary trailing zeros while preserving precision
        std::snprintf(buf, sizeof(buf), "%.8g", static_cast<double>(v));
        out += buf;
    }

    void boolean(bool v) { raw(v ? "true" : "false"); }
    void null_val()      { raw("null"); }

    void uint32(uint32_t v) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%u", v);
        out += buf;
    }

    void vec3(const vm::Vector3<float>& v) {
        raw('['); num(v.x()); raw(','); num(v.y()); raw(','); num(v.z()); raw(']');
    }

    void quat(const vm::Quaternion<float>& q) {
        raw('['); num(q.x()); raw(','); num(q.y()); raw(','); num(q.z()); raw(','); num(q.w()); raw(']');
    }

    void begin_obj() { raw('{'); ++indent; nl(); }
    void end_obj()   { --indent; nl(); raw('}'); }
    void begin_arr() { raw('['); ++indent; nl(); }
    void end_arr()   { --indent; nl(); raw(']'); }
};

void writeShapeBody(Writer& w, const Shape* shape) {
    if (!shape) { w.null_val(); return; }
    switch (shape->type()) {
    case ShapeType::Sphere: {
        const auto* s = static_cast<const SphereShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("sphere"); w.raw(','); w.nl();
        w.key("radius"); w.num(s->radius());
        w.end_obj();
        break;
    }
    case ShapeType::Box: {
        const auto* b = static_cast<const BoxShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("box"); w.raw(','); w.nl();
        w.key("halfExtents"); w.vec3(b->halfExtents());
        w.end_obj();
        break;
    }
    case ShapeType::Capsule: {
        const auto* c = static_cast<const CapsuleShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("capsule"); w.raw(','); w.nl();
        w.key("radius"); w.num(c->radius()); w.raw(','); w.nl();
        w.key("halfHeight"); w.num(c->halfHeight());
        w.end_obj();
        break;
    }
    case ShapeType::Cylinder: {
        const auto* cy = static_cast<const CylinderShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("cylinder"); w.raw(','); w.nl();
        w.key("radius"); w.num(cy->radius()); w.raw(','); w.nl();
        w.key("halfHeight"); w.num(cy->halfHeight());
        w.end_obj();
        break;
    }
    case ShapeType::ConvexHull: {
        const auto* ch = static_cast<const ConvexHullShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("convexHull"); w.raw(','); w.nl();
        w.key("points"); w.begin_arr();
        const auto& pts = ch->points();
        for (std::size_t i = 0; i < pts.size(); ++i) {
            w.vec3(pts[i]);
            if (i + 1 < pts.size()) { w.raw(','); w.nl(); }
        }
        w.end_arr();
        w.end_obj();
        break;
    }
    case ShapeType::Compound: {
        const auto* comp = static_cast<const CompoundShape*>(shape);
        w.begin_obj();
        w.key("type"); w.str("compound"); w.raw(','); w.nl();
        w.key("children"); w.begin_arr();
        const auto& kids = comp->children();
        for (std::size_t i = 0; i < kids.size(); ++i) {
            w.begin_obj();
            w.key("shape"); writeShapeBody(w, kids[i].shape.get()); w.raw(','); w.nl();
            w.key("position"); w.vec3(kids[i].localTransform.position); w.raw(','); w.nl();
            w.key("rotation"); w.quat(kids[i].localTransform.rotation);
            w.end_obj();
            if (i + 1 < kids.size()) { w.raw(','); w.nl(); }
        }
        w.end_arr();
        w.end_obj();
        break;
    }
    case ShapeType::TriangleMesh:
        w.begin_obj();
        w.key("type"); w.str("triangleMesh");
        w.end_obj();
        break;
    case ShapeType::HeightField:
        w.begin_obj();
        w.key("type"); w.str("heightField");
        w.end_obj();
        break;
    default:
        w.null_val();
        break;
    }
}

static const char* bodyTypeName(BodyType t) {
    switch (t) {
    case BodyType::Static:    return "static";
    case BodyType::Kinematic: return "kinematic";
    case BodyType::Dynamic:   return "dynamic";
    case BodyType::Sensor:    return "sensor";
    default:                  return "dynamic";
    }
}

} // namespace

// ══════════════════════════════════════════════════════════════════════════════
// PhysicsWorld::serialize
// ══════════════════════════════════════════════════════════════════════════════

std::string PhysicsWorld::serialize() const {
    Writer w;
    w.begin_obj();

    w.key("version"); w.raw("1"); w.raw(','); w.nl();

    // World settings
    w.key("world"); w.begin_obj();
    w.key("gravity"); w.vec3(m_integratorSettings.gravity); w.raw(','); w.nl();
    w.key("fixedTimestep"); w.num(m_fixedTimestep); w.raw(','); w.nl();
    w.key("substeps"); w.out += std::to_string(m_substeps); w.raw(','); w.nl();
    w.key("workerThreads"); w.out += std::to_string(m_workerThreads); w.raw(','); w.nl();
    w.key("contactIterations"); w.out += std::to_string(contactIterations); w.raw(','); w.nl();
    w.key("constraintIterations"); w.out += std::to_string(constraintIterations); w.raw(','); w.nl();
    w.key("contactBaumgarte"); w.num(contactBaumgarte); w.raw(','); w.nl();
    w.key("contactSlop"); w.num(contactSlop);
    w.end_obj();
    w.raw(','); w.nl();

    // Bodies
    w.key("bodies"); w.begin_arr();
    bool firstBody = true;
    m_pool.forEach([&](uint32_t id, const BodyData& d) {
        if (!firstBody) { w.raw(','); w.nl(); }
        firstBody = false;

        w.begin_obj();
        w.key("id"); w.uint32(id); w.raw(','); w.nl();
        w.key("type"); w.str(bodyTypeName(d.type)); w.raw(','); w.nl();

        // Transform
        w.key("position"); w.vec3(d.transform.position); w.raw(','); w.nl();
        w.key("rotation"); w.quat(d.transform.rotation); w.raw(','); w.nl();

        // Kinematics
        w.key("linearVelocity");  w.vec3(d.linearVelocity);  w.raw(','); w.nl();
        w.key("angularVelocity"); w.vec3(d.angularVelocity); w.raw(','); w.nl();

        // Mass (0 for static/kinematic/sensor with invMass == 0)
        const float mass = (d.invMass > 1e-30f) ? 1.f / d.invMass : 0.f;
        w.key("mass"); w.num(mass); w.raw(','); w.nl();

        // Material / damping
        w.key("linearDamping");  w.num(d.linearDamping);  w.raw(','); w.nl();
        w.key("angularDamping"); w.num(d.angularDamping); w.raw(','); w.nl();
        w.key("restitution");    w.num(d.restitution);    w.raw(','); w.nl();
        w.key("friction");       w.num(d.friction);       w.raw(','); w.nl();

        // Collision filtering
        w.key("layer"); w.uint32(d.layer); w.raw(','); w.nl();
        w.key("mask");  w.uint32(d.mask);  w.raw(','); w.nl();

        // Flags
        w.key("ccdEnabled");  w.boolean(d.ccdEnabled);  w.raw(','); w.nl();
        w.key("isSleeping");  w.boolean(d.isSleeping);  w.raw(','); w.nl();

        // Shape
        w.key("shape"); writeShapeBody(w, d.shape.get());

        w.end_obj();
    });
    w.end_arr();

    w.end_obj();
    return std::move(w.out);
}

// ══════════════════════════════════════════════════════════════════════════════
// Minimal JSON parser
// ══════════════════════════════════════════════════════════════════════════════

namespace {

// Forward-declared so JVal can hold std::vector<JKV> with an incomplete element
// type — std::vector allows this in C++17, but std::pair does not.
struct JKV;

struct JVal {
    enum Kind { Null, Bool, Number, Str, Arr, Obj } kind = Null;
    bool              b{};
    double            n{};
    std::string       s;
    std::vector<JVal> arr;
    std::vector<JKV>  obj;  // JKV is incomplete here; methods that use it are defined below

    // Defined after JKV is complete (see below).
    const JVal* get(std::string_view key) const;

    float            asFloat (float    def = 0.f)  const { return kind == Number ? static_cast<float>(n)    : def; }
    double           asDouble(double   def = 0.0)  const { return kind == Number ? n                        : def; }
    bool             asBool  (bool     def = false) const { return kind == Bool   ? b                        : def; }
    uint32_t         asUint  (uint32_t def = 0)    const { return kind == Number ? static_cast<uint32_t>(n) : def; }
    int              asInt   (int      def = 0)    const { return kind == Number ? static_cast<int>(n)      : def; }
    std::string_view asStr   ()                    const { return kind == Str    ? std::string_view(s)      : std::string_view{}; }
};

struct JKV { std::string key; JVal val; };

inline const JVal* JVal::get(std::string_view key) const {
    if (kind != Obj) return nullptr;
    for (const auto& kv : obj)
        if (kv.key == key) return &kv.val;
    return nullptr;
}

struct Parser {
    std::string_view src;
    std::size_t      pos = 0;

    char peek() const { return pos < src.size() ? src[pos] : '\0'; }
    char next()       { return pos < src.size() ? src[pos++] : '\0'; }
    bool done() const { return pos >= src.size(); }

    void skipWs() {
        while (!done() && (src[pos]==' '||src[pos]=='\t'||src[pos]=='\n'||src[pos]=='\r'))
            ++pos;
    }

    bool expect(char c) {
        skipWs();
        if (peek() == c) { ++pos; return true; }
        return false;
    }

    std::optional<JVal> parseValue();
    std::optional<std::string> parseString();
    std::optional<JVal> parseNumber();
    std::optional<JVal> parseObject();
    std::optional<JVal> parseArray();

    bool ok = true;
    void fail() { ok = false; }
};

std::optional<std::string> Parser::parseString() {
    if (!expect('"')) { fail(); return std::nullopt; }
    std::string result;
    while (!done()) {
        char c = next();
        if (c == '"') return result;
        if (c == '\\') {
            char e = next();
            switch (e) {
            case '"':  result += '"';  break;
            case '\\': result += '\\'; break;
            case '/':  result += '/';  break;
            case 'n':  result += '\n'; break;
            case 'r':  result += '\r'; break;
            case 't':  result += '\t'; break;
            default:   result += e;    break;
            }
        } else {
            result += c;
        }
    }
    fail(); return std::nullopt;
}

std::optional<JVal> Parser::parseNumber() {
    skipWs();
    std::size_t start = pos;
    if (peek() == '-') ++pos;
    while (!done() && (std::isdigit(static_cast<unsigned char>(peek())) ||
                       peek() == '.' || peek() == 'e' || peek() == 'E' ||
                       peek() == '+' || peek() == '-'))
        ++pos;
    if (pos == start) { fail(); return std::nullopt; }
    JVal v;
    v.kind = JVal::Number;
    char* end = nullptr;
    v.n = std::strtod(src.data() + start, &end);
    return v;
}

std::optional<JVal> Parser::parseObject() {
    if (!expect('{')) { fail(); return std::nullopt; }
    JVal v; v.kind = JVal::Obj;
    skipWs();
    if (peek() == '}') { ++pos; return v; }
    while (!done()) {
        skipWs();
        auto ks = parseString();
        if (!ks || !ok) { fail(); return std::nullopt; }
        if (!expect(':')) { fail(); return std::nullopt; }
        auto val = parseValue();
        if (!val || !ok) { fail(); return std::nullopt; }
        v.obj.push_back({std::move(*ks), std::move(*val)});
        skipWs();
        if (peek() == '}') { ++pos; return v; }
        if (!expect(',')) { fail(); return std::nullopt; }
    }
    fail(); return std::nullopt;
}

std::optional<JVal> Parser::parseArray() {
    if (!expect('[')) { fail(); return std::nullopt; }
    JVal v; v.kind = JVal::Arr;
    skipWs();
    if (peek() == ']') { ++pos; return v; }
    while (!done()) {
        auto val = parseValue();
        if (!val || !ok) { fail(); return std::nullopt; }
        v.arr.push_back(std::move(*val));
        skipWs();
        if (peek() == ']') { ++pos; return v; }
        if (!expect(',')) { fail(); return std::nullopt; }
    }
    fail(); return std::nullopt;
}

std::optional<JVal> Parser::parseValue() {
    skipWs();
    char c = peek();
    if (c == '{') return parseObject();
    if (c == '[') return parseArray();
    if (c == '"') {
        auto s = parseString();
        if (!s) return std::nullopt;
        JVal v; v.kind = JVal::Str; v.s = std::move(*s); return v;
    }
    if (c == 't') { pos+=4; JVal v; v.kind=JVal::Bool; v.b=true;  return v; }
    if (c == 'f') { pos+=5; JVal v; v.kind=JVal::Bool; v.b=false; return v; }
    if (c == 'n') { pos+=4; JVal v; v.kind=JVal::Null;             return v; }
    if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parseNumber();
    fail(); return std::nullopt;
}

// ── Shape reconstruction ──────────────────────────────────────────────────────

std::shared_ptr<Shape> shapeFromJson(const JVal& jsh);

vm::Vector3<float> vec3FromArr(const JVal& j, vm::Vector3<float> def = {}) {
    if (j.kind != JVal::Arr || j.arr.size() < 3) return def;
    return vm::Vector3<float>(j.arr[0].asFloat(), j.arr[1].asFloat(), j.arr[2].asFloat());
}

vm::Quaternion<float> quatFromArr(const JVal& j) {
    if (j.kind != JVal::Arr || j.arr.size() < 4)
        return vm::Quaternion<float>::identity();
    return vm::Quaternion<float>(
        j.arr[0].asFloat(), j.arr[1].asFloat(),
        j.arr[2].asFloat(), j.arr[3].asFloat());
}

std::shared_ptr<Shape> shapeFromJson(const JVal& j) {
    if (j.kind == JVal::Null) return nullptr;
    if (j.kind != JVal::Obj)  return nullptr;

    const JVal* jtype = j.get("type");
    if (!jtype || jtype->kind != JVal::Str) return nullptr;
    std::string_view t = jtype->asStr();

    if (t == "sphere") {
        float r = 0.5f;
        if (const auto* jr = j.get("radius")) r = jr->asFloat(0.5f);
        return std::make_shared<SphereShape>(r);
    }
    if (t == "box") {
        vm::Vector3<float> he(0.5f, 0.5f, 0.5f);
        if (const auto* jhe = j.get("halfExtents")) he = vec3FromArr(*jhe, he);
        return std::make_shared<BoxShape>(he);
    }
    if (t == "capsule") {
        float r  = 0.25f, hh = 0.5f;
        if (const auto* jr  = j.get("radius"))     r  = jr->asFloat(r);
        if (const auto* jhh = j.get("halfHeight")) hh = jhh->asFloat(hh);
        return std::make_shared<CapsuleShape>(r, hh);
    }
    if (t == "cylinder") {
        float r  = 0.25f, hh = 0.5f;
        if (const auto* jr  = j.get("radius"))     r  = jr->asFloat(r);
        if (const auto* jhh = j.get("halfHeight")) hh = jhh->asFloat(hh);
        return std::make_shared<CylinderShape>(r, hh);
    }
    if (t == "convexHull") {
        std::vector<vm::Vector3<float>> pts;
        if (const auto* jpts = j.get("points")) {
            if (jpts->kind == JVal::Arr) {
                pts.reserve(jpts->arr.size());
                for (const auto& p : jpts->arr)
                    pts.push_back(vec3FromArr(p));
            }
        }
        return std::make_shared<ConvexHullShape>(std::move(pts));
    }
    if (t == "compound") {
        auto comp = std::make_shared<CompoundShape>();
        if (const auto* jkids = j.get("children")) {
            if (jkids->kind == JVal::Arr) {
                for (const auto& child : jkids->arr) {
                    Transform lt;
                    if (const auto* jp = child.get("position"))
                        lt.position = vec3FromArr(*jp);
                    if (const auto* jr = child.get("rotation"))
                        lt.rotation = quatFromArr(*jr);
                    std::shared_ptr<Shape> cs;
                    if (const auto* jcs = child.get("shape"))
                        cs = shapeFromJson(*jcs);
                    if (cs) comp->addChild(std::move(cs), lt);
                }
            }
        }
        return comp;
    }
    // triangleMesh / heightField: not reconstructable — return null
    return nullptr;
}

static BodyType bodyTypeFromStr(std::string_view s) {
    if (s == "static")    return BodyType::Static;
    if (s == "kinematic") return BodyType::Kinematic;
    if (s == "sensor")    return BodyType::Sensor;
    return BodyType::Dynamic;
}

} // namespace

// ══════════════════════════════════════════════════════════════════════════════
// PhysicsWorld::deserialize
// ══════════════════════════════════════════════════════════════════════════════

bool PhysicsWorld::deserialize(std::string_view json) {
    // Destroy all existing bodies (also cleans up broadphase proxies)
    {
        std::vector<uint32_t> ids;
        m_pool.forEach([&](uint32_t id, const BodyData&) { ids.push_back(id); });
        for (uint32_t id : ids) destroyBody(m_pool.makeHandle(id));
    }

    Parser p;
    p.src = json;
    auto root = p.parseValue();
    if (!p.ok || !root || root->kind != JVal::Obj) return false;

    // World settings
    if (const auto* jw = root->get("world")) {
        if (const auto* jg = jw->get("gravity"))
            m_integratorSettings.gravity = vec3FromArr(*jg, {0.f,-9.81f,0.f});
        if (const auto* jft = jw->get("fixedTimestep"))
            m_fixedTimestep = jft->asFloat(1.f / 60.f);
        if (const auto* js = jw->get("substeps"))
            m_substeps = js->asInt(1);
        if (const auto* jt = jw->get("workerThreads"))
            m_workerThreads = std::max(1, jt->asInt(1));
        if (const auto* jci = jw->get("contactIterations"))
            contactIterations = jci->asInt(10);
        if (const auto* jki = jw->get("constraintIterations"))
            constraintIterations = jki->asInt(10);
        if (const auto* jcb = jw->get("contactBaumgarte"))
            contactBaumgarte = jcb->asFloat(0.2f);
        if (const auto* jcs = jw->get("contactSlop"))
            contactSlop = jcs->asFloat(0.005f);
    }

    // Bodies
    const JVal* jbodies = root->get("bodies");
    if (!jbodies || jbodies->kind != JVal::Arr) return true; // empty world is OK

    for (const auto& jb : jbodies->arr) {
        if (jb.kind != JVal::Obj) continue;

        BodyDescriptor desc;

        if (const auto* jt = jb.get("type"))
            desc.type = bodyTypeFromStr(jt->asStr());

        // Transform
        if (const auto* jp = jb.get("position"))
            desc.transform.position = vec3FromArr(*jp);
        if (const auto* jr = jb.get("rotation"))
            desc.transform.rotation = quatFromArr(*jr);

        // Kinematics
        if (const auto* jlv = jb.get("linearVelocity"))
            desc.linearVelocity = vec3FromArr(*jlv);
        if (const auto* jav = jb.get("angularVelocity"))
            desc.angularVelocity = vec3FromArr(*jav);

        if (const auto* jm = jb.get("mass"))
            desc.mass = jm->asFloat(1.f);

        // Material
        if (const auto* jld = jb.get("linearDamping"))
            desc.linearDamping  = jld->asFloat(0.01f);
        if (const auto* jad = jb.get("angularDamping"))
            desc.angularDamping = jad->asFloat(0.05f);
        if (const auto* jre = jb.get("restitution"))
            desc.restitution = jre->asFloat(0.3f);
        if (const auto* jfr = jb.get("friction"))
            desc.friction = jfr->asFloat(0.5f);

        // Filtering
        if (const auto* jl = jb.get("layer"))
            desc.layer = jl->asUint(0xFFFFFFFF);
        if (const auto* jmk = jb.get("mask"))
            desc.mask = jmk->asUint(0xFFFFFFFF);

        // Flags
        if (const auto* jccd = jb.get("ccdEnabled"))
            desc.ccdEnabled = jccd->asBool(false);

        // Shape
        if (const auto* jsh = jb.get("shape"))
            desc.shape = shapeFromJson(*jsh);

        Body body = createBody(desc);

        // Restore sleep state after creation (createBody wakes by default)
        if (const auto* jsl = jb.get("isSleeping"))
            if (jsl->asBool(false)) m_pool.get(body.id()).isSleeping = true;
    }

    return true;
}

} // namespace campello::physics
