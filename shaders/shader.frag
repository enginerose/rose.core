#version 450
layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec2 vUv;
layout(location = 2) in vec4 vClipPos;
layout(location = 3) in vec4 vPrevClipPos;

layout(set = 0, binding = 0) uniform sampler2D uBaseColor;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 MotionVector;

const vec3  kSunDir  = vec3(0.4, 1.0, 0.3);
const float kAmbient = 0.25;
const float kDiffuse = 0.75;

void main() {
    vec3  n = normalize(vWorldNormal);
    float diffuse = max(dot(n, normalize(kSunDir)), 0.0) * kDiffuse;
    float lighting = kAmbient + diffuse;
    vec3 baseColor = texture(uBaseColor, vUv).rgb;

    FragColor = vec4(baseColor * lighting, 1.0);

    vec2 currentUv = (vClipPos.xy / vClipPos.w) * 0.5 + 0.5;
    vec2 previousUv = (vPrevClipPos.xy / vPrevClipPos.w) * 0.5 + 0.5;
    MotionVector = currentUv - previousUv;
}
