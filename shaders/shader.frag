#version 330 core
in vec3 vNormal;
in vec3 vUv;

out vec4 FragColor;

void main() {
    vec3 baseColor = normalize(abs(vNormal));
    FragColor = vec4(baseColor, 1.0);
}