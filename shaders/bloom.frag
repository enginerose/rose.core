#version 450

layout(location = 0) in vec2 vUv;

layout(set = 0, binding = 0) uniform sampler2D uSceneColor;

layout(push_constant) uniform BloomPushConstants {
    vec2 uTexelSize;
    float uThreshold;
    float uIntensity;
    float uRadius;
    int uQuality;
} pc;

layout(location = 0) out vec4 FragColor;

const float kPi = 3.14159265359;
const float kLumaR = 0.2126;
const float kLumaG = 0.7152;
const float kLumaB = 0.0722;

vec3 brightPart(vec3 color) {
    float luma = dot(color, vec3(kLumaR, kLumaG, kLumaB));
    float amount = max(luma - pc.uThreshold, 0.0) / max(luma, 0.0001);
    return color * amount;
}

void main() {
    vec3 baseColor = texture(uSceneColor, vUv).rgb;
    vec3 bloom = brightPart(baseColor);
    float weightSum = 1.0;

    int quality = clamp(pc.uQuality, 1, 64);
    for (int i = 0; i < quality; ++i) {
        float t = (float(i) + 0.5) / float(quality);
        float angle = t * kPi * 2.39996323;
        float radius = sqrt(t) * pc.uRadius;
        vec2 offset = vec2(cos(angle), sin(angle)) * radius * pc.uTexelSize;
        float weight = 1.0 - t * 0.65;
        bloom += brightPart(texture(uSceneColor, vUv + offset).rgb) * weight;
        weightSum += weight;
    }

    FragColor = vec4(baseColor + bloom * (pc.uIntensity / weightSum), 1.0);
}
