#version 450
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;

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

layout(location = 0) out vec3 vWorldNormal;
layout(location = 1) out vec2 vUv;
layout(location = 2) out vec4 vClipPos;
layout(location = 3) out vec4 vPrevClipPos;

vec4 toVulkanClip(vec4 clipPos) {
    clipPos.y = -clipPos.y;
    clipPos.z = (clipPos.z + clipPos.w) * 0.5;
    return clipPos;
}

void main() {
    vWorldNormal = normalize(mat3(transpose(inverse(pc.uModel))) * aNormal);
    vUv = aUv;

    vec4 worldPos = pc.uModel * vec4(aPos, 1.0);
    if (pc.uOutlineEnabled != 0) {
        vec3 worldCenter = (pc.uModel * vec4(pc.uOutlineCenter, 1.0)).xyz;
        vec3 expandDir = worldPos.xyz - worldCenter;
        if (dot(expandDir, expandDir) < 0.000001) {
            expandDir = vWorldNormal;
        }
        worldPos.xyz += normalize(expandDir) * pc.uOutlineWidth;
    }

    vec4 clipPos = toVulkanClip(pc.uMVP * worldPos);
    vec4 prevClipPos = toVulkanClip(pc.uPrevMVP * worldPos);
    vClipPos = clipPos;
    vPrevClipPos = prevClipPos;
    gl_Position = clipPos;
}
