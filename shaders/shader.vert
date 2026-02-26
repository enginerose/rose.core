#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aUv;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vNormal;
out vec2 vUv;

void main() {
    vNormal = aNormal;
    vUv = aUv;
    gl_Position = uMVP * uModel * vec4(aPos, 1.0);
}