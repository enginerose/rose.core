#version 450
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;

layout(push_constant) uniform PushConstants {
    mat4 uMVP;
    mat4 uModel;
    mat4 uPrevMVP;
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
    vWorldNormal = mat3(transpose(inverse(pc.uModel))) * aNormal;
    vUv = aUv;

    vec4 clipPos = toVulkanClip(pc.uMVP * pc.uModel * vec4(aPos, 1.0));
    vec4 prevClipPos = toVulkanClip(pc.uPrevMVP * pc.uModel * vec4(aPos, 1.0));
    vClipPos = clipPos;
    vPrevClipPos = prevClipPos;
    gl_Position = clipPos;
}
