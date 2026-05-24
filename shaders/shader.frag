#version 450
layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec2 vUv;
layout(location = 2) in vec4 vClipPos;
layout(location = 3) in vec4 vPrevClipPos;
layout(location = 4) in vec3 vWorldPos;
layout(location = 5) in vec4 vShadowClipPos;
layout(location = 6) in vec4 vSunShadowClipPos;

layout(set = 0, binding = 0) uniform sampler2D uBaseColor;
layout(set = 0, binding = 1) uniform sampler2D uNormal;
layout(set = 0, binding = 2) uniform sampler2D uMetallicRoughness;
layout(set = 0, binding = 3) uniform sampler2D uEmissive;
layout(set = 0, binding = 4) uniform MaterialParams {
    vec4 uBaseColorFactor;
    vec4 uEmissiveFactor;
    float uMetallicFactor;
    float uRoughnessFactor;
    float uNormalScale;
    float uPadding;
} material;

layout(set = 1, binding = 0) uniform LightParams {
    vec4 uPositionEnabled;
    vec4 uDirection;
    vec4 uColorIntensity;
    vec4 uParams;
    mat4 uViewProjection;
    vec4 uSunDirectionEnabled;
    vec4 uSunColorIntensity;
    vec4 uSunParams;
    mat4 uSunViewProjection;
} light;
layout(set = 1, binding = 1) uniform sampler2D uShadowMap;
layout(set = 1, binding = 2) uniform sampler2D uSunShadowMap;

layout(push_constant) uniform PushConstants {
    mat4 uMVP;
    mat4 uModel;
    mat4 uPrevMVP;
    vec3 uOutlineCenter;
    float uOutlineWidth;
    vec3 uOutlineColor;
    float uOutlineAlpha;
    int uOutlineEnabled;
    vec3 uOutlinePadding;
} pc;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 MotionVector;

const float kPi = 3.14159265359;
const vec3 kAmbient = vec3(0.035);

vec3 srgbToLinear(vec3 color) {
    bvec3 cutoff = lessThanEqual(color, vec3(0.04045));
    vec3 lower = color / 12.92;
    vec3 higher = pow((color + 0.055) / 1.055, vec3(2.4));
    return mix(higher, lower, cutoff);
}

float distributionGGX(vec3 n, vec3 h, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float nDotH = max(dot(n, h), 0.0);
    float nDotH2 = nDotH * nDotH;
    float denom = nDotH2 * (a2 - 1.0) + 1.0;
    return a2 / max(kPi * denom * denom, 0.0001);
}

float geometrySchlickGGX(float nDotV, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return nDotV / max(nDotV * (1.0 - k) + k, 0.0001);
}

float geometrySmith(vec3 n, vec3 v, vec3 l, float roughness) {
    return geometrySchlickGGX(max(dot(n, v), 0.0), roughness)
         * geometrySchlickGGX(max(dot(n, l), 0.0), roughness);
}

vec3 fresnelSchlick(float cosTheta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 evaluatePbrLight(vec3 n, vec3 v, vec3 l, vec3 radiance, vec3 albedo, float metallic, float roughness, vec3 f0) {
    vec3 h = normalize(v + l);
    float nDotL = max(dot(n, l), 0.0);
    float nDotV = max(dot(n, v), 0.0001);
    vec3 f = fresnelSchlick(max(dot(h, v), 0.0), f0);
    float d = distributionGGX(n, h, roughness);
    float g = geometrySmith(n, v, l, roughness);

    vec3 specular = (d * g * f) / max(4.0 * nDotV * nDotL, 0.0001);
    vec3 diffuse = (vec3(1.0) - f) * (1.0 - metallic) * albedo / kPi;
    return (diffuse + specular) * radiance * nDotL;
}

mat3 cotangentFrame(vec3 n, vec3 p, vec2 uv) {
    vec3 dp1 = dFdx(p);
    vec3 dp2 = dFdy(p);
    vec2 duv1 = dFdx(uv);
    vec2 duv2 = dFdy(uv);
    vec3 dp2perp = cross(dp2, n);
    vec3 dp1perp = cross(n, dp1);
    vec3 t = dp2perp * duv1.x + dp1perp * duv2.x;
    vec3 b = dp2perp * duv1.y + dp1perp * duv2.y;
    float invMax = inversesqrt(max(max(dot(t, t), dot(b, b)), 0.000001));
    return mat3(t * invMax, b * invMax, n);
}

vec3 pbrNormal() {
    vec3 n = normalize(vWorldNormal);
    vec3 tangentNormal = texture(uNormal, vUv).xyz * 2.0 - 1.0;
    tangentNormal.xy *= material.uNormalScale;
    return normalize(cotangentFrame(n, vWorldPos, vUv) * normalize(tangentNormal));
}

float shadowVisibility(vec4 shadowClipPos, sampler2D shadowMap, vec3 n, vec3 l, float baseBias) {
    vec3 shadowNdc = shadowClipPos.xyz / shadowClipPos.w;
    vec2 shadowUv = shadowNdc.xy * 0.5 + 0.5;
    if (shadowUv.x < 0.0 || shadowUv.x > 1.0
        || shadowUv.y < 0.0 || shadowUv.y > 1.0
        || shadowNdc.z < 0.0 || shadowNdc.z > 1.0) {
        return 1.0;
    }

    float bias = max(baseBias * (1.0 - dot(n, l)), baseBias * 0.35);
    vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    float visible = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            float closestDepth = texture(shadowMap, shadowUv + vec2(x, y) * texelSize).r;
            visible += shadowNdc.z - bias <= closestDepth ? 1.0 : 0.0;
        }
    }
    return mix(0.18, 1.0, visible / 9.0);
}

void main() {
    if (pc.uOutlineEnabled != 0) {
        FragColor = vec4(pc.uOutlineColor, pc.uOutlineAlpha);
        MotionVector = vec2(0.0);
        return;
    }

    vec4 baseSample = texture(uBaseColor, vUv);
    vec3 albedo = srgbToLinear(baseSample.rgb) * material.uBaseColorFactor.rgb;
    float alpha = baseSample.a * material.uBaseColorFactor.a;

    vec4 metallicRoughnessSample = texture(uMetallicRoughness, vUv);
    float roughness = clamp(metallicRoughnessSample.g * material.uRoughnessFactor, 0.04, 1.0);
    float metallic = clamp(metallicRoughnessSample.b * material.uMetallicFactor, 0.0, 1.0);
    vec3 emissive = srgbToLinear(texture(uEmissive, vUv).rgb) * material.uEmissiveFactor.rgb;

    vec3 n = pbrNormal();
    vec3 v = normalize(pc.uOutlineCenter - vWorldPos);
    vec3 light_delta = light.uPositionEnabled.xyz - vWorldPos;
    float light_distance = length(light_delta);
    vec3 l = light_delta / max(light_distance, 0.0001);

    float spotCos = dot(normalize(-l), normalize(light.uDirection.xyz));
    float spot = smoothstep(light.uParams.y, light.uParams.x, spotCos);
    float range = max(light.uParams.z, 0.0001);
    float rangeAttenuation = clamp(1.0 - (light_distance * light_distance) / (range * range), 0.0, 1.0);
    rangeAttenuation *= rangeAttenuation;
    float distanceAttenuation = 1.0 / max(light_distance * light_distance, 1.0);
    float shadow = shadowVisibility(vShadowClipPos, uShadowMap, n, l, light.uParams.w);
    vec3 radiance = light.uColorIntensity.rgb
                  * light.uColorIntensity.a
                  * light.uPositionEnabled.w
                  * spot
                  * rangeAttenuation
                  * distanceAttenuation
                  * shadow;
    vec3 f0 = mix(vec3(0.04), albedo, metallic);
    vec3 sunL = normalize(-light.uSunDirectionEnabled.xyz);
    float sunShadow = shadowVisibility(vSunShadowClipPos, uSunShadowMap, n, sunL, light.uSunParams.x);
    vec3 sunRadiance = light.uSunColorIntensity.rgb
                     * light.uSunColorIntensity.a
                     * light.uSunDirectionEnabled.w
                     * sunShadow;
    vec3 color = evaluatePbrLight(n, v, l, radiance, albedo, metallic, roughness, f0)
               + evaluatePbrLight(n, v, sunL, sunRadiance, albedo, metallic, roughness, f0)
               + albedo * kAmbient * (1.0 - metallic)
               + emissive;

    FragColor = vec4(color, alpha);

    vec2 currentUv = (vClipPos.xy / vClipPos.w) * 0.5 + 0.5;
    vec2 previousUv = (vPrevClipPos.xy / vPrevClipPos.w) * 0.5 + 0.5;
    MotionVector = currentUv - previousUv;
}
