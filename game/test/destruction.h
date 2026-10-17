#pragma once
// Polygonal local CSG (solid minus oriented polygonal drill).
// No voxel conversion: original box, pyramid, or sphere triangles are kept
// wherever they do not intersect the cutter. Cap and wall polygons are clipped
// against the original convex mesh and existing cuts.
#include "shape.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <glm/gtc/constants.hpp>
#include <vector>

namespace Destruction {
struct Plane { glm::vec3 n; float d; }; // material inside: dot(n,p) <= d
using Polygon = std::vector<glm::vec3>;
inline float sqLength(const glm::vec3& p) { return glm::dot(p,p); }

inline void appendTriangle(std::vector<glm::vec3>& mesh,
                           const glm::vec3& a, const glm::vec3& b,
                           const glm::vec3& c) {
    if (sqLength(glm::cross(b-a,c-a)) < 1e-12f) return;
    mesh.insert(mesh.end(), {a,b,c});
}
inline void appendPolygon(std::vector<glm::vec3>& mesh, const Polygon& poly) {
    if (poly.size() < 3) return;
    for (std::size_t i=1; i+1<poly.size(); ++i)
        appendTriangle(mesh, poly[0],poly[i],poly[i+1]);
}
inline void appendOutward(std::vector<glm::vec3>& m,
                          const glm::vec3& a, const glm::vec3& b,
                          const glm::vec3& c) {
    if (glm::dot(glm::cross(b-a,c-a), (a+b+c)*(1.0f/3.0f)) < 0.0f)
        appendTriangle(m,a,c,b);
    else appendTriangle(m,a,b,c);
}
inline std::vector<glm::vec3> originalTriangles(const Shape& shape) {
    std::vector<glm::vec3> mesh;
    const glm::vec3 h = glm::abs(shape.scale)*0.5f;
    if(shape.type==ShapeType::Rect) {
        const glm::vec3 p[8] = {
            {-h.x,-h.y,-h.z}, {h.x,-h.y,-h.z}, {h.x,h.y,-h.z}, {-h.x,h.y,-h.z},
            {-h.x,-h.y,h.z}, {h.x,-h.y,h.z}, {h.x,h.y,h.z}, {-h.x,h.y,h.z}};
        constexpr int q[6][4]={{0,3,2,1},{4,5,6,7},{0,4,7,3},
                               {1,2,6,5},{0,1,5,4},{3,7,6,2}};
        for (const auto& f:q) {
            appendOutward(mesh,p[f[0]],p[f[1]],p[f[2]]);
            appendOutward(mesh,p[f[0]],p[f[2]],p[f[3]]);
        }
    } else if(shape.type==ShapeType::Pyramid) {
        const glm::vec3 p[5]={ {-h.x,-h.y,-h.z}, {h.x,-h.y,-h.z},
            {h.x,-h.y,h.z}, {-h.x,-h.y,h.z}, {0,h.y,0} };
        for(int i=0;i<4;++i) appendOutward(mesh,p[i],p[(i+1)%4],p[4]);
        appendOutward(mesh,p[0],p[2],p[1]);
        appendOutward(mesh,p[0],p[3],p[2]);
    } else if(shape.type==ShapeType::Sphere) {
        // Same 20x20 tessellation and axis convention as Renderer::init().
        const glm::vec3 r=glm::abs(shape.scale);
        constexpr int sectors=20, stacks=20;
        std::array<std::array<glm::vec3,sectors+1>,stacks+1> p{};
        for(int i=0;i<=stacks;++i) {
            const float ang=glm::pi<float>()*0.5f - i*glm::pi<float>()/stacks;
            for(int j=0;j<=sectors;++j) {
                const float sector= j*2.0f*glm::pi<float>()/sectors;
                p[i][j]=glm::vec3(std::cos(ang)*std::cos(sector),
                                  std::cos(ang)*std::sin(sector),
                                  std::sin(ang))*r;
            }
        }
        for(int i=0;i<stacks;++i) for(int j=0;j<sectors;++j){
            const auto& a=p[i][j]; const auto& b=p[i+1][j];
            const auto& c=p[i][j+1]; const auto& d=p[i+1][j+1];
            if(i!=0) appendOutward(mesh,a,b,c);
            if(i!=stacks-1) appendOutward(mesh,c,b,d);
        }
    }
    return mesh;
}

inline std::vector<Plane> convexPlanes(const std::vector<glm::vec3>& original) {
    std::vector<Plane> result;
    for (std::size_t i=0;i+2<original.size();i+=3) {
        const glm::vec3& a=original[i];
        glm::vec3 n=glm::cross(original[i+1]-a, original[i+2]-a);
        const float len=glm::length(n);
        if(len < 1e-7f) continue;
        n/=len;
        const float d=glm::dot(n,a);
        bool duplicate=false;
        for(const Plane& p:result) if(glm::dot(p.n,n)>0.99999f && std::abs(p.d-d)<0.00005f){duplicate=true;break;}
        if(!duplicate) result.push_back({n,d});
    }
    return result;
}

inline Polygon clipHalfspace(const Polygon& input,const Plane& plane,bool inside) {
    Polygon out;
    if(input.empty()) return out;
    constexpr float eps=1.e-5f;
    auto signedDistance = [&](const glm::vec3& p) {
        const float d=glm::dot(p,plane.n)-plane.d;
        return inside ? d : -d;
    };
    glm::vec3 prev=input.back();
    float dp=signedDistance(prev);
    for(const glm::vec3& current:input) {
        const float dc=signedDistance(current);
        const bool pin=dp<=eps, cin=dc<=eps;
        if(pin!=cin) {
            const float t=std::clamp(dp/(dp-dc),0.0f,1.0f);
            out.push_back(prev+(current-prev)*t);
        }
        if(cin) out.push_back(current);
        prev=current; dp=dc;
    }
    return out;
}
inline Polygon insideConvex(Polygon p,const std::vector<Plane>& planes) {
    for(const Plane& plane:planes){
        p=clipHalfspace(p,plane,true);
        if(p.size()<3) break;
    }
    return p;
}
// Polygon MINUS a convex cutter. The inside remainder is discarded;
// outside regions are emitted independently (works with holes in polygons).
inline void outsideConvex(const Polygon& input,const std::vector<Plane>& planes,
                          std::vector<glm::vec3>& output) {
    // Reject unchanged faces before entering the expensive partitioner.
    // Otherwise the infinite cutter planes subdivide faces far from the hole.
    for(const Plane& plane:planes) {
        bool whollyOutside=true;
        for(const glm::vec3& vertex:input)
            if(glm::dot(plane.n,vertex)<=plane.d+1e-5f){whollyOutside=false;break;}
        if(whollyOutside){appendPolygon(output,input);return;}
    }
    Polygon remaining=input;
    for(const Plane& plane:planes) {
        if(remaining.size()<3) break;
        appendPolygon(output,clipHalfspace(remaining,plane,false));
        remaining=clipHalfspace(remaining,plane,true);
    }
}
inline std::vector<Plane> cutterPlanes(const CavityCut& cut) {
    std::vector<Plane> planes;
    const glm::vec3 n=glm::normalize(cut.normal);
    planes.push_back({n,glm::dot(n,cut.point+n*cut.outside)});
    planes.push_back({-n,glm::dot(-n,cut.point-n*cut.depth)});
    const glm::vec3 u=glm::normalize(glm::cross(n,std::abs(n.y)<0.95f
                                     ? glm::vec3(0,1,0):glm::vec3(1,0,0)));
    const glm::vec3 v=glm::cross(n,u);
    constexpr int sides=16;
    for(int i=0;i<sides;++i){
        const float a=2.0f*glm::pi<float>()*i/sides;
        const float b=2.0f*glm::pi<float>()*(i+1)/sides;
        const glm::vec3 ra=u*std::cos(a)+v*std::sin(a);
        const glm::vec3 rb=u*std::cos(b)+v*std::sin(b);
        const glm::vec3 radial=glm::normalize(ra+rb);
        planes.push_back({radial,glm::dot(radial,cut.point+ra*cut.radius)});
    }
    return planes;
}
inline std::vector<glm::vec3> cutterTriangles(const CavityCut& cut) {
    std::vector<glm::vec3> result;
    const glm::vec3 n=glm::normalize(cut.normal);
    const glm::vec3 u=glm::normalize(glm::cross(n,std::abs(n.y)<0.95f
                                     ? glm::vec3(0,1,0):glm::vec3(1,0,0)));
    const glm::vec3 v=glm::cross(n,u);
    constexpr int sides=16;
    const glm::vec3 front=cut.point+n*cut.outside;
    const glm::vec3 back=cut.point-n*cut.depth;
    for(int i=0;i<sides;++i) {
        const float a=2.0f*glm::pi<float>()*i/sides;
        const float b=2.0f*glm::pi<float>()*(i+1)/sides;
        const glm::vec3 ra=(u*std::cos(a)+v*std::sin(a))*cut.radius;
        const glm::vec3 rb=(u*std::cos(b)+v*std::sin(b))*cut.radius;
        // Reverse cutter face normals: these are outward from remaining solid.
        appendTriangle(result,front+ra,front+rb,back+rb);
        appendTriangle(result,front+ra,back+rb,back+ra);
        appendTriangle(result,back,back+ra,back+rb); // inward into removed channel
        appendTriangle(result,front,front+rb,front+ra);
    }
    return result;
}
// Build a per-shape local-space BVH after the CPU mesh changes. The tree
// is rebuilt only on a shot, never once per physics frame or raycast.
inline void buildBVH(MeshDamage& mesh) {
    const int count=static_cast<int>(mesh.triangleBounds.size());
    mesh.triangleOrder.resize(count);
    for(int i=0;i<count;++i)mesh.triangleOrder[i]=i;
    mesh.bvh.clear();
    mesh.bvh.reserve(static_cast<std::size_t>(count)*2);
    if(count==0)return;
    std::function<int(int,int)> build=[&](int begin,int end)->int {
        const int index=static_cast<int>(mesh.bvh.size());
        mesh.bvh.emplace_back();
        glm::vec3 mn(1e30f),mx(-1e30f);
        for(int i=begin;i<end;++i){
            const auto& b=mesh.triangleBounds[mesh.triangleOrder[i]];
            mn=glm::min(mn,b.min); mx=glm::max(mx,b.max);
        }
        mesh.bvh[index].min=mn;
        mesh.bvh[index].max=mx;
        if(end-begin<=8){
            mesh.bvh[index].first=begin;
            mesh.bvh[index].count=end-begin;
            return index;
        }
        const glm::vec3 dimensions=mx-mn;
        const int axis=(dimensions.x>dimensions.y && dimensions.x>dimensions.z)
                         ?0:(dimensions.y>dimensions.z?1:2);
        const int middle=(begin+end)/2;
        std::nth_element(mesh.triangleOrder.begin()+begin,
                         mesh.triangleOrder.begin()+middle,
                         mesh.triangleOrder.begin()+end,[&](int x,int y){
            const auto& a=mesh.triangleBounds[x];
            const auto& b=mesh.triangleBounds[y];
            return (a.min[axis]+a.max[axis])<(b.min[axis]+b.max[axis]);
        });
        const int left=build(begin,middle);
        const int right=build(middle,end);
        mesh.bvh[index].left=left;
        mesh.bvh[index].right=right;
        return index;
    };
    build(0,count);
}

inline bool insideCut(const CavityCut& cut,const glm::vec3& p) {
    for(const Plane& plane:cutterPlanes(cut)) if(glm::dot(plane.n,p)>plane.d+1e-5f) return false;
    return true;
}
inline bool insideOriginal(const std::vector<Plane>& planes,const glm::vec3& p){
    for(const Plane& plane:planes) if(glm::dot(plane.n,p)>plane.d+1e-4f) return false;
    return true;
}
struct CarveResult {
    std::vector<glm::vec3> removedLocalCenters;
    bool changed() const {return !removedLocalCenters.empty();}
};
inline CarveResult carve(Shape& shape, const glm::vec3& worldPoint,
                         const glm::vec3& worldNormal,float radius=0.33f,
                         float depth=0.55f) {
    if(shape.isShotFragment || shape.id==100) return {};
    if(shape.type!=ShapeType::Rect && shape.type!=ShapeType::Pyramid &&
       shape.type!=ShapeType::Sphere) return {};
    const glm::quat inv=glm::inverse(shape.rot);
    CavityCut cut;
    cut.point=inv*(worldPoint-shape.pos);
    cut.normal=glm::normalize(inv*worldNormal);
    cut.radius=radius;cut.depth=depth;
    const auto original=originalTriangles(shape);
    const auto basePlanes=convexPlanes(original);
    if(original.empty() || basePlanes.empty()) return {};
    const auto planes=cutterPlanes(cut);

    CarveResult result;
    const glm::vec3 n=cut.normal;
    const glm::vec3 u=glm::normalize(glm::cross(n,std::abs(n.y)<0.95f
                                     ? glm::vec3(0,1,0):glm::vec3(1,0,0)));
    const glm::vec3 v=glm::cross(n,u);
    // Only spawn debris where material actually existed before this cut.
    for(int j=0;j<8;++j) for(int k=0;k<3;++k) {
        const float a=2.0f*glm::pi<float>()*(j+0.5f)/8.0f;
        const float r=radius*(k==0?0.2f:(k==1?0.52f:0.77f));
        const glm::vec3 sample=cut.point-n*(depth*0.36f)+
                    r*(std::cos(a)*u+std::sin(a)*v);
        if(!insideOriginal(basePlanes,sample)) continue;
        bool previouslyRemoved=false;
        for(const auto& old:shape.damage.cuts) if(insideCut(old,sample)){previouslyRemoved=true;break;}
        if(!previouslyRemoved) result.removedLocalCenters.push_back(sample);
    }
    // Surface clipping also covers very thin shapes where depth sample misses.
    if(result.removedLocalCenters.empty()) {
        const glm::vec3 sample=cut.point-n*std::min(depth*0.14f,0.02f);
        if(insideOriginal(basePlanes,sample)) {
            bool old=false;
            for(const auto& prev:shape.damage.cuts) if(insideCut(prev,sample)) old=true;
            if(!old) result.removedLocalCenters.push_back(sample);
        }
    }
    if(!result.changed()) return {};

    std::vector<glm::vec3> clipped;
    const auto& current=shape.damage.active()?shape.damage.triangles:original;
    clipped.reserve(current.size()+512);
    for(std::size_t i=0;i+2<current.size();i+=3)
        outsideConvex({current[i],current[i+1],current[i+2]},planes,clipped);

    // Add the cutter's inner walls and back face, clipped to the untouched
    // convex parent AND excluding all previously drilled voids.
    const auto cutterMesh=cutterTriangles(cut);
    for(std::size_t i=0;i+2<cutterMesh.size();i+=3) {
        Polygon p=insideConvex({cutterMesh[i],cutterMesh[i+1],cutterMesh[i+2]},basePlanes);
        if(p.size()<3) continue;
        std::vector<Polygon> parts{std::move(p)};
        for(const CavityCut& old:shape.damage.cuts) {
            std::vector<glm::vec3> outside;
            const auto oldPlanes=cutterPlanes(old);
            for(const Polygon& fragment:parts) outsideConvex(fragment,oldPlanes,outside);
            parts.clear();
            for(std::size_t j=0;j+2<outside.size();j+=3)
                parts.push_back({outside[j],outside[j+1],outside[j+2]});
            if(parts.empty()) break;
        }
        for(const Polygon& fragment:parts) appendPolygon(clipped,fragment);
    }
    // Avoid unbounded CPU/GPU growth from pathological repeated overlapping cuts.
    constexpr std::size_t maximumVertices=90000;
    if(clipped.size()>maximumVertices) return {};
    shape.damage.triangles=std::move(clipped);
    shape.damage.triangleBounds.clear();
    shape.damage.triangleBounds.reserve(shape.damage.triangles.size()/3);
    for(std::size_t i=0;i+2<shape.damage.triangles.size();i+=3) {
        const glm::vec3 a=shape.damage.triangles[i];
        const glm::vec3 b=shape.damage.triangles[i+1];
        const glm::vec3 c=shape.damage.triangles[i+2];
        shape.damage.triangleBounds.push_back({glm::min(a,glm::min(b,c)),
                                               glm::max(a,glm::max(b,c))});
    }
    buildBVH(shape.damage);
    shape.damage.cuts.push_back(cut);
    ++shape.damage.revision;
    return result;
}
} // namespace Destruction
