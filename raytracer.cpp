#include <iostream> 
#include <cmath>
#include <cfloat>
#include <thread>
#include "parser.h" 
#include "ppm.h"    

using namespace parser;
using namespace std;

typedef int ObjectType;

const ObjectType NONE     = 0;
const ObjectType TRIANGLE = 1;
const ObjectType MESH     = 2;
const ObjectType SPHERE   = 3;

typedef unsigned char RGB[3];

class Ray{
public:
    Vec3f origin;
    Vec3f direction;
    
    Ray() {}
    Ray(const Vec3f& o, const Vec3f& d) : origin(o), direction(d) {}
};

class HitInfo{
public:
    bool doesIntersect;
    float t;

    HitInfo() : doesIntersect(false), t(DBL_MAX) {}
};

class ObjectInfo{
public:
    int objID;
    ObjectType objType;

    ObjectInfo() : objID(-1), objType(NONE) {}
};

class SurfaceInfo{
public:
    Vec3f surfaceNormal;
    Vec3f intersectPoint;

    SurfaceInfo() {}
};

Vec3f add(const Vec3f &v1, const Vec3f &v2){
    Vec3f v3;
    v3.x = v1.x + v2.x;
    v3.y = v1.y + v2.y;
    v3.z = v1.z + v2.z;
    return v3;
}

Vec3f subtract(const Vec3f &v1, const Vec3f &v2){
    Vec3f v3;
    v3.x = v1.x - v2.x;
    v3.y = v1.y - v2.y;
    v3.z = v1.z - v2.z;
    return v3;
}

float findLength(const Vec3f &v){
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

float findDistance(const Vec3f &v1, const Vec3f &v2){
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2));
}

Vec3f crossProduct(const Vec3f &v1, const Vec3f &v2){
    Vec3f result;
    result.x = v1.y*v2.z - v1.z*v2.y;
    result.y = v1.z*v2.x - v1.x*v2.z;
    result.z = v1.x*v2.y - v1.y*v2.x;
    return result;
}

float dotProduct(const Vec3f &v1, const Vec3f &v2){
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

Vec3f normalize(const Vec3f &vec){
    Vec3f result;
    float len = findLength(vec);
    if(len != 0){
        result.x = vec.x / len;
        result.y = vec.y / len;
        result.z = vec.z / len;
    } 
    else{
        result = vec;
    }
    return result;
}

Vec3f multiplyVectorWithConst(const Vec3f &v, float c){
    Vec3f result;
    result.x = v.x * c;
    result.y = v.y * c;
    result.z = v.z * c;
    return result;
}

Vec3f findIntersectionPoint(const Ray &ray, float t){
    Vec3f point;
    point.x = ray.origin.x + t*ray.direction.x;
    point.y = ray.origin.y + t*ray.direction.y;
    point.z = ray.origin.z + t*ray.direction.z;
    return point;
}

Ray generateRay(const Camera &camera, int i, int j){
    Ray ray;
    ray.origin = camera.position;

    Vec3f gaze = camera.gaze;
    float left = camera.near_plane.x;
    float right = camera.near_plane.y;
    float bottom = camera.near_plane.z;
    float top = camera.near_plane.w;

    Vec3f u = crossProduct(gaze, camera.up);
    u = normalize(u);
    Vec3f v = normalize(camera.up);

    Vec3f m = add(ray.origin, multiplyVectorWithConst(gaze, camera.near_distance));
    Vec3f q = add(m, add(multiplyVectorWithConst(u, left), multiplyVectorWithConst(v, top)));

    float nx = camera.image_width;
    float ny = camera.image_height;
    float su = (right - left) * ((i + 0.5) / nx);
    float sv = (top - bottom) * ((j + 0.5) / ny);
    
    Vec3f s = add(q, subtract(multiplyVectorWithConst(u, su), multiplyVectorWithConst(v, sv)));
    ray.direction = normalize(subtract(s, ray.origin));
    return ray;
}

float determinant(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2){
    return v0.x*(v1.y*v2.z - v1.z*v2.y) - v0.y*(v1.x*v2.z - v1.z*v2.x) + v0.z*(v1.x*v2.y - v1.y*v2.x);
}

void triangleIntersection(const Scene &scene, const Ray &ray, const Triangle &triangle, 
                         HitInfo &hit, ObjectInfo &objectInfo, SurfaceInfo &surface){
    Vec3f v_a = scene.vertex_data[triangle.indices.v0_id - 1];
    Vec3f v_b = scene.vertex_data[triangle.indices.v1_id - 1];
    Vec3f v_c = scene.vertex_data[triangle.indices.v2_id - 1];

    float a = v_a.x - v_b.x;
    float b = v_a.y - v_b.y;
    float c = v_a.z - v_b.z;
    float d = v_a.x - v_c.x;
    float e = v_a.y - v_c.y;
    float f = v_a.z - v_c.z;
    float g = ray.direction.x;
    float h = ray.direction.y;
    float i_ = ray.direction.z;

    float detA = a*(e*i_ - h*f) - b*(d*i_ - f*g) + c*(d*h - e*g);
    if(detA == 0) return;

    float val = determinant(subtract(v_a, v_b), subtract(v_a, v_c), subtract(v_a, ray.origin)) / detA;
    float gamma = determinant(subtract(v_a, v_b), subtract(v_a, ray.origin), ray.direction) / detA;
    float beta = determinant(subtract(v_a, ray.origin), subtract(v_a, v_c), ray.direction) / detA;

    if(beta >= 0 && gamma >= 0 && (beta + gamma) <= 1 && val > 0){
        hit.doesIntersect = true;
        hit.t = val;
        objectInfo.objType = TRIANGLE;
        surface.surfaceNormal = normalize(crossProduct(subtract(v_c, v_b), subtract(v_a, v_b)));
        surface.intersectPoint = findIntersectionPoint(ray, val);
    }
}

void meshIntersection(const Scene &scene, const Ray &ray, const Mesh &mesh, 
                     HitInfo &hit, ObjectInfo &objectInfo, SurfaceInfo &surface){
    Triangle curr;
    Vec3f normal;
    bool flag = false;
    float tmin = DBL_MAX;
    for(const auto &mesh_face : mesh.faces){
        curr.material_id = mesh.material_id;
        curr.indices = mesh_face;
        HitInfo tempHit;
        ObjectInfo tempObject;
        SurfaceInfo tempSurface;
        triangleIntersection(scene, ray, curr, tempHit, tempObject, tempSurface);
        
        if(tempHit.doesIntersect && tempHit.t < tmin && tempHit.t > 0.0){
            tmin = tempHit.t;
            flag = tempHit.doesIntersect;
            normal = tempSurface.surfaceNormal;
            objectInfo.objID = mesh_face.v0_id; 
            objectInfo.objType = MESH;
        }
    }
    if(flag){
        hit.doesIntersect = true;
        hit.t = tmin;
        surface.surfaceNormal = normal;
        surface.intersectPoint = findIntersectionPoint(ray, tmin);
    }
}

void sphereIntersection(const Scene &scene, const Ray &ray, const Sphere &sphere, 
                       HitInfo &hit, ObjectInfo &objectInfo, SurfaceInfo &surface){
    Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];

    float A = dotProduct(ray.direction, ray.direction);
    float B = 2 * dotProduct(ray.direction, subtract(ray.origin, center));
    float C = dotProduct(subtract(ray.origin, center), subtract(ray.origin, center)) - sphere.radius * sphere.radius;
    float discriminant = B*B - 4*A*C;
    float val;

    if(discriminant < 0){
        hit.doesIntersect = false;
        return;
    }
    else if (discriminant == 0){
        val = -B / (2*A);
        if(val > 0){
            hit.t = val;
            hit.doesIntersect = true;
        }
    }
    else{
        float t1 = (-B + sqrt(discriminant)) / (2*A);
        float t2 = (-B - sqrt(discriminant)) / (2*A);

        float temp = (t1 < t2 && t1 > 0) ? t1 : ((t2 > 0) ? t2 : -1);
        
        if(temp >= 0.0){
            val = temp;
            hit.t = val;
            hit.doesIntersect = true;
            objectInfo.objType = SPHERE;
            surface.intersectPoint = findIntersectionPoint(ray, val);
            Vec3f p_minus_c = subtract(surface.intersectPoint, center);
            surface.surfaceNormal = normalize(p_minus_c); 
        }
        else{
            hit.t = -1;
            hit.doesIntersect = false;
        }
    }
}

Vec3f diffuseShading(const Scene &scene, const SurfaceInfo &surface, int materialId, const PointLight &light){
    Vec3f diffuse;

    Vec3f difReflectance = scene.materials[materialId - 1].diffuse;

    Vec3f wiN = normalize(subtract(light.position, surface.intersectPoint));

    Vec3f lightIntensity = light.intensity;
    float r = findDistance(light.position, surface.intersectPoint);
    r = r * r;

    lightIntensity.x = lightIntensity.x / r;
    lightIntensity.y = lightIntensity.y / r;
    lightIntensity.z = lightIntensity.z / r;

    float wiN_times_n = dotProduct(surface.surfaceNormal, wiN);

    if(wiN_times_n < 0.0){
        wiN_times_n = 0.0;
    }

    diffuse.x = difReflectance.x * wiN_times_n * lightIntensity.x;
    diffuse.y = difReflectance.y * wiN_times_n * lightIntensity.y;
    diffuse.z = difReflectance.z * wiN_times_n * lightIntensity.z;

    return diffuse;
}

Vec3f specularLight(const Scene &scene, const SurfaceInfo &surface, int materialId, const PointLight &light, const Vec3f &camPosition){
    Vec3f specular;

    Vec3f specReflectance = scene.materials[materialId - 1].specular;

    Vec3f halfVector;
    Vec3f V = normalize(subtract(camPosition, surface.intersectPoint));
    Vec3f L = normalize(subtract(light.position, surface.intersectPoint));

    Vec3f N = surface.surfaceNormal;

    halfVector = normalize(add(L, V));

    float half_dot_normal = dotProduct(halfVector, N);
    
    if(half_dot_normal < 0.0){
        half_dot_normal = 0.0;
    }

    float phong = scene.materials[materialId - 1].phong_exponent;

    half_dot_normal = pow(half_dot_normal, phong);

    Vec3f lightIntensity = light.intensity;
    float r = findDistance(light.position, surface.intersectPoint);
    r = r * r;

    lightIntensity.x = lightIntensity.x / r;   
    lightIntensity.y = lightIntensity.y / r;
    lightIntensity.z = lightIntensity.z / r;

    specular.x = specReflectance.x * half_dot_normal * lightIntensity.x;
    specular.y = specReflectance.y * half_dot_normal * lightIntensity.y;
    specular.z = specReflectance.z * half_dot_normal * lightIntensity.z;

    return specular;
}

Ray reflectionRay(const Scene &scene, const SurfaceInfo &surface, const Vec3f &camPosition){
    Ray reflection;

    Vec3f w_0 = normalize(subtract(camPosition, surface.intersectPoint));
    Vec3f N = surface.surfaceNormal;
    float n_dot_w0 = dotProduct(N, w_0);
    w_0 = multiplyVectorWithConst(w_0, -1.0);

    Vec3f w_r = normalize(add(w_0, multiplyVectorWithConst(N, 2*n_dot_w0)));

    reflection.direction = w_r;
    reflection.origin = add(multiplyVectorWithConst(N, scene.shadow_ray_epsilon), surface.intersectPoint);

    return reflection;
}

void findIntersectionResult(const Scene &scene, const Ray &ray, 
                           HitInfo &closestHit, ObjectInfo &closestObject, SurfaceInfo &closestSurface){
    closestHit.t = DBL_MAX;
    closestHit.doesIntersect = false;
    closestObject = ObjectInfo();
    closestSurface = SurfaceInfo();

    int sphereSize = scene.spheres.size();
    int triangleSize = scene.triangles.size();
    int meshSize = scene.meshes.size();

    for(int i = 0; i < triangleSize; i++){
        HitInfo tempHit;
        ObjectInfo tempObject;
        SurfaceInfo tempSurface;
        triangleIntersection(scene, ray, scene.triangles[i], tempHit, tempObject, tempSurface);
        if(tempHit.doesIntersect && tempHit.t < closestHit.t && tempHit.t > 0.0){
            closestHit = tempHit;
            closestObject = tempObject;
            closestObject.objID = i;
            closestSurface = tempSurface;
        }
    }

    for(int i = 0; i < meshSize; i++){
        HitInfo tempHit;
        ObjectInfo tempObject;
        SurfaceInfo tempSurface;
        meshIntersection(scene, ray, scene.meshes[i], tempHit, tempObject, tempSurface);
        if(tempHit.doesIntersect && tempHit.t < closestHit.t && tempHit.t > 0.0){
            closestHit = tempHit;
            closestObject = tempObject;
            closestObject.objID = i;
            closestSurface = tempSurface;
        }
    }

    for(int i = 0; i < sphereSize; i++){
        HitInfo tempHit;
        ObjectInfo tempObject;
        SurfaceInfo tempSurface;
        sphereIntersection(scene, ray, scene.spheres[i], tempHit, tempObject, tempSurface);
        if(tempHit.doesIntersect && tempHit.t < closestHit.t && tempHit.t > 0.0){
            closestHit = tempHit;
            closestObject = tempObject;
            closestObject.objID = i;
            closestSurface = tempSurface;
        }
    }
}

bool isMirror(const Material &material){
    return !(material.mirror.x == 0 && material.mirror.y == 0 && material.mirror.z == 0);
}

bool isInShadow(const Scene &scene, const Vec3f &vec, const Vec3f &toLight){
    Ray shadowRay;
    shadowRay.origin = vec;
    Vec3f toLightNormalized = normalize(toLight);
    shadowRay.direction = toLightNormalized;

    HitInfo shadowHit;
    ObjectInfo shadowObject;
    SurfaceInfo shadowSurface;
    findIntersectionResult(scene, shadowRay, shadowHit, shadowObject, shadowSurface);

    float toLightLength = findLength(toLight);
    float shadowObjLength = findLength(subtract(shadowSurface.intersectPoint, vec));
    return (shadowHit.doesIntersect && (shadowObjLength < toLightLength));
}

Vec3f calculateLightContribution(const Scene &scene, const SurfaceInfo &surface, int materialId, const Vec3f &camPosition){
    Vec3f pixelValue = {0.0, 0.0, 0.0};

    for(const auto &light : scene.point_lights){
        Vec3f diff = diffuseShading(scene, surface, materialId, light);
        Vec3f spec = specularLight(scene, surface, materialId, light, camPosition);
        Vec3f toLight = subtract(light.position, surface.intersectPoint);

        Vec3f total = add(surface.intersectPoint, multiplyVectorWithConst(surface.surfaceNormal, scene.shadow_ray_epsilon));
        if(isInShadow(scene, total, toLight)){
            continue;
        }
        pixelValue = add(pixelValue, add(diff, spec));
    }
    return pixelValue;
}

Vec3f calculateCamPosition(const Vec3f &camPosition, const SurfaceInfo &intersection, const Vec3f &reflectedDirection){
    Vec3f distanceVector = subtract(camPosition, intersection.intersectPoint);
    float distance = findLength(distanceVector);
    return add(multiplyVectorWithConst(reflectedDirection, distance), intersection.intersectPoint);
}

Vec3f calculatePixelColor(const Scene &scene, const SurfaceInfo &closestSurface, const ObjectInfo &closestObject, const Vec3f &camPosition, int recursionDepth);

Vec3f calculateReflection(const Scene &scene, const SurfaceInfo &intersection, const ObjectInfo &objectInfo, int materialID, const Vec3f &camPosition, int recursionDepth){
    Vec3f result = {0.0, 0.0, 0.0};
    if(recursionDepth > 0 && isMirror(scene.materials[materialID-1])){
        Ray reflectRay = reflectionRay(scene, intersection, camPosition);  
        HitInfo reflectHit;
        ObjectInfo reflectObject;
        SurfaceInfo reflectSurface;
        findIntersectionResult(scene, reflectRay, reflectHit, reflectObject, reflectSurface);
        if(reflectHit.doesIntersect){
            
            Vec3f reflectedDirection = multiplyVectorWithConst(reflectRay.direction, -1);
            Vec3f newPosition = calculateCamPosition(camPosition, intersection, reflectedDirection);
            Vec3f reflectionPixelColor = calculatePixelColor(scene, reflectSurface, reflectObject, newPosition, recursionDepth - 1);

            result.x += reflectionPixelColor.x * scene.materials[materialID-1].mirror.x;
            result.y += reflectionPixelColor.y * scene.materials[materialID-1].mirror.y;
            result.z += reflectionPixelColor.z * scene.materials[materialID-1].mirror.z;
            
        }
    }
    return result;
}

Vec3f calculateAmbientColor(const Material &material, const Vec3f &ambientLight){
    return Vec3f{
        material.ambient.x * ambientLight.x,
        material.ambient.y * ambientLight.y,
        material.ambient.z * ambientLight.z
    };
}

int getMaterialID(const Scene &scene, const ObjectInfo &objectInfo){
    if (objectInfo.objType == SPHERE){
        return scene.spheres[objectInfo.objID].material_id;
    } 
    else if (objectInfo.objType == MESH){
        return scene.meshes[objectInfo.objID].material_id;
    } 
    else if (objectInfo.objType == TRIANGLE){
        return scene.triangles[objectInfo.objID].material_id;
    } 
    else{
        return -1;
    }
}

Vec3f calculatePixelColor(const Scene &scene, const SurfaceInfo &closestSurface, const ObjectInfo &closestObject, const Vec3f &camPosition, int recursionDepth){
    Vec3f pixelColor = {0.0, 0.0, 0.0};
    
    if(closestObject.objType != NONE){
        int materialID = getMaterialID(scene, closestObject);
        if (materialID < 0){ 
            pixelColor.x = scene.background_color.x;
            pixelColor.y = scene.background_color.y;
            pixelColor.z = scene.background_color.z;
            return pixelColor;
        }

        pixelColor = add(pixelColor, calculateAmbientColor(scene.materials[materialID - 1], scene.ambient_light));
        pixelColor = add(pixelColor, calculateLightContribution(scene, closestSurface, materialID, camPosition));
        pixelColor = add(pixelColor, calculateReflection(scene, closestSurface, closestObject, materialID, camPosition, recursionDepth));
                
    }
    else{
        pixelColor.x = scene.background_color.x;
        pixelColor.y = scene.background_color.y;
        pixelColor.z = scene.background_color.z;
    }

    return pixelColor;
}

void clampColor(Vec3f &rgb){
    rgb.x = (rgb.x > 255) ? 255 : rgb.x;
    rgb.y = (rgb.y > 255) ? 255 : rgb.y;
    rgb.z = (rgb.z > 255) ? 255 : rgb.z;
}

void setPixelColor(unsigned char* image, int &pixelNum, const Vec3f &rgb){
    image[pixelNum++] = static_cast<unsigned char>(round(rgb.x));
    image[pixelNum++] = static_cast<unsigned char>(round(rgb.y));
    image[pixelNum++] = static_cast<unsigned char>(round(rgb.z));
}

void createImageLines(const Scene &scene, int start, int end, int imageWidth, int camNum, unsigned char* image){
    int pixelNum = start * imageWidth * 3;
    for(int j = start; j < end; j++){
        for(int i = 0; i < imageWidth; i++){
            Ray currRay = generateRay(scene.cameras[camNum], i, j);
            HitInfo closestHit;
            ObjectInfo closestObject;
            SurfaceInfo closestSurface;
            findIntersectionResult(scene, currRay, closestHit, closestObject, closestSurface);
            Vec3f pixel = calculatePixelColor(scene, closestSurface, closestObject, scene.cameras[camNum].position, scene.max_recursion_depth);
            clampColor(pixel);
            setPixelColor(image, pixelNum, pixel);
        }
    }
}

void processCameraImage(Scene &scene, int camIndex){
    int imageWidthPixels = scene.cameras[camIndex].image_width;
    int imageHeightPixels = scene.cameras[camIndex].image_height;

    unsigned char* image = new unsigned char[imageWidthPixels * imageHeightPixels * 3];

    int threadCount = std::thread::hardware_concurrency();
    int rowsPerThread = imageHeightPixels / threadCount;

    std::vector<std::thread> threads;
    for (int t = 0; t < threadCount; ++t){
        int startRow = t * rowsPerThread;
        int endRow = (t == threadCount - 1) ? imageHeightPixels : (t + 1) * rowsPerThread;
        threads.emplace_back(createImageLines, std::ref(scene), startRow, endRow, imageWidthPixels, camIndex, image);
    }

    for (auto& t : threads){
        t.join();
    }

    write_ppm(scene.cameras[camIndex].image_name.c_str(), image, imageWidthPixels, imageHeightPixels);
    delete[] image;
}

int main(int argc, char* argv[]){

    Scene scene;
    scene.loadFromXml(argv[1]);

    for(int i=0; i<scene.cameras.size(); i++){
        processCameraImage(scene, i);
    }
    return 0;
}
