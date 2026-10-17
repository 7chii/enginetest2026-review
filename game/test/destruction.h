#pragma once
// Polygonal local CSG (solid minus oriented hemispherical cutter).
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
// Tessellate a convex hemispherical cutter once per carve/query. Its opening
// is a circular mouth on the impact plane, and the curved cup extends inward.
// The short outward collar is intentionally OUTSIDE the shape: it avoids a
// coplanar cutter cap on the original surface (which would seal the hole).
//
// The same cutter triangles generate BOTH the CSG planes and the triangles
// placed on the inner walls. This keeps rendering, raycasts and collision
// clipping consistent even when two hits overlap.
inline std::vector<glm::vec3> cutterOutwardTriangles(const CavityCut& cut) {
    constexpr int sectors = 20;
    constexpr int curvedBands = 6;
    const glm::vec3 n = glm::normalize(cut.normal);
    const glm::vec3 u = glm::normalize(glm::cross(n,
        std::abs(n.y) < 0.95f ? glm::vec3(0,1,0) : glm::vec3(1,0,0)));
    const glm::vec3 v = glm::cross(n,u);
    const glm::vec3 interior = cut.point - n * (cut.depth * 0.25f);
    std::vector<glm::vec3> triangles;
    triangles.reserve(static_cast<std::size_t>(sectors) * (curvedBands * 6 + 12));

    // Winding must be outward from the removed volume. The final cavity
    // surface uses the reverse winding to point out of the remaining solid.
    auto addOutward = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        if (glm::dot(glm::cross(b-a,c-a),(a+b+c)/3.0f-interior) < 0.0f)
            std::swap(b,c);
        appendTriangle(triangles,a,b,c);
    };
    auto pointOnRing = [&](int band, int sector) -> glm::vec3 {
        const float theta = 0.5f * glm::pi<float>() *
                            static_cast<float>(band) / curvedBands;
        const float phi = 2.0f * glm::pi<float>() *
                          static_cast<float>(sector) / sectors;
        const glm::vec3 radial = u * std::cos(phi) + v * std::sin(phi);
        // Equal radius/depth is a true hemisphere. Explicit unequal values
        // make a half-ellipsoid without changing the CSG algorithm.
        return cut.point + radial * (cut.radius * std::cos(theta))
                         - n * (cut.depth * std::sin(theta));
    };
    const glm::vec3 tip = cut.point - n * cut.depth;
    const glm::vec3 capCenter = cut.point + n * cut.outside;
    for (int sector=0; sector<sectors; ++sector) {
        const int next = (sector+1)%sectors;
        const glm::vec3 a = pointOnRing(0,sector);
        const glm::vec3 b = pointOnRing(0,next);
        const glm::vec3 frontA = a + n * cut.outside;
        const glm::vec3 frontB = b + n * cut.outside;
        // Collar and front cap close the cutter *outside* the parent shape.
        addOutward(frontA,frontB,b);
        addOutward(frontA,b,a);
        addOutward(capCenter,frontA,frontB);
        for (int band=0;band<curvedBands-1;++band) {
            const glm::vec3 upperA=pointOnRing(band,sector);
            const glm::vec3 upperB=pointOnRing(band,next);
            const glm::vec3 lowerA=pointOnRing(band+1,sector);
            const glm::vec3 lowerB=pointOnRing(band+1,next);
            addOutward(upperA,upperB,lowerB);
            addOutward(upperA,lowerB,lowerA);
        }
        addOutward(pointOnRing(curvedBands-1,sector),
                   pointOnRing(curvedBands-1,next),tip);
    }
    return triangles;
}
inline std::vector<Plane> cutterPlanes(const CavityCut& cut) {
    // Cutter faces are emitted in adjacent, coplanar triangle pairs. Keep
    // one plane per facet instead of O(facets^2) duplicate comparisons:
    // physics also calls this for collision clipping while objects move.
    const auto faces = cutterOutwardTriangles(cut);
    std::vector<Plane> planes;
    planes.reserve(144);
    const glm::vec3 axis=glm::normalize(cut.normal);
    const float frontD=glm::dot(axis,cut.point+axis*cut.outside);
    bool frontAdded=false;
    for(std::size_t i=0;i+2<faces.size();i+=3) {
        const glm::vec3& a=faces[i];
        glm::vec3 n=glm::cross(faces[i+1]-a,faces[i+2]-a);
        const float length=glm::length(n);
        if(length<1.e-7f) continue;
        n/=length;
        const float d=glm::dot(n,a);
        if(glm::dot(n,axis)>0.99999f && std::abs(d-frontD)<0.00005f) {
            if(frontAdded) continue;
            frontAdded=true;
        }
        if(!planes.empty() && glm::dot(planes.back().n,n)>0.99999f &&
           std::abs(planes.back().d-d)<0.00005f) continue;
        planes.push_back({n,d});
    }
    return planes;
}
inline std::vector<glm::vec3> cutterTriangles(const CavityCut& cut) {
    std::vector<glm::vec3> triangles = cutterOutwardTriangles(cut);
    for (std::size_t i=0; i+2<triangles.size(); i+=3)
        std::swap(triangles[i+1],triangles[i+2]);
    return triangles;
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

inline bool insidePlanes(const std::vector<Plane>& planes,const glm::vec3& p) {
    for(const Plane& plane:planes)
        if(glm::dot(plane.n,p)>plane.d+1e-5f) return false;
    return true;
}
inline bool insideCut(const CavityCut& cut,const glm::vec3& p) {
    return insidePlanes(cutterPlanes(cut),p);
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
                         float depth=-1.0f) {
    if(shape.isShotFragment || shape.id==100) return {};
    if(shape.type!=ShapeType::Rect && shape.type!=ShapeType::Pyramid &&
       shape.type!=ShapeType::Sphere) return {};
    const glm::quat inv=glm::inverse(shape.rot);
    CavityCut cut;
    cut.point=inv*(worldPoint-shape.pos);
    cut.normal=glm::normalize(inv*worldNormal);
    if (!std::isfinite(radius) || radius <= 0.0f ||
        (depth != -1.0f && (!std::isfinite(depth) || depth <= 0.0f))) return {};
    cut.radius=radius;
    cut.depth=depth > 0.0f ? depth : radius;
    const auto original=originalTriangles(shape);
    const auto basePlanes=convexPlanes(original);
    if(original.empty() || basePlanes.empty()) return {};
    const auto planes=cutterPlanes(cut);
    // Reuse old clipping planes across every new bowl triangle and sample.
    // Generating them per triangle would be costly with a curved cutter.
    std::vector<std::vector<Plane>> oldCutPlanes;
    oldCutPlanes.reserve(shape.damage.cuts.size());
    for(const CavityCut& old:shape.damage.cuts)
        oldCutPlanes.push_back(cutterPlanes(old));

    CarveResult result;
    const glm::vec3 n=cut.normal;
    const glm::vec3 u=glm::normalize(glm::cross(n,std::abs(n.y)<0.95f
                                     ? glm::vec3(0,1,0):glm::vec3(1,0,0)));
    const glm::vec3 v=glm::cross(n,u);
    // Only spawn debris where material actually existed before this cut.
    for(int j=0;j<8;++j) for(int k=0;k<3;++k) {
        const float a=2.0f*glm::pi<float>()*(j+0.5f)/8.0f;
        const float r=radius*(k==0?0.2f:(k==1?0.52f:0.77f));
        const glm::vec3 sample=cut.point-n*(cut.depth*0.36f)+
                    r*(std::cos(a)*u+std::sin(a)*v);
        if(!insideOriginal(basePlanes,sample)) continue;
        bool previouslyRemoved=false;
        for(const auto& oldPlanes:oldCutPlanes) if(insidePlanes(oldPlanes,sample)){previouslyRemoved=true;break;}
        if(!previouslyRemoved) result.removedLocalCenters.push_back(sample);
    }
    // Surface clipping also covers very thin shapes where depth sample misses.
    if(result.removedLocalCenters.empty()) {
        const glm::vec3 sample=cut.point-n*std::min(cut.depth*0.14f,0.02f);
        if(insideOriginal(basePlanes,sample)) {
            bool old=false;
            for(const auto& oldPlanes:oldCutPlanes) if(insidePlanes(oldPlanes,sample)) old=true;
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
        for(const auto& oldPlanes:oldCutPlanes) {
            std::vector<glm::vec3> outside;
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
