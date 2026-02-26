#version 330 core
in vec3 vNormal;
in vec2 vUv;

uniform sampler2D uBaseColor;
uniform bool uHasTexture;

out vec4 FragColor;

void main() {
    if (uHasTexture) {
        FragColor = texture(uBaseColor, vUv);
    } else {
        vec3 baseColor = normalize(abs(vNormal));
        FragColor = vec4(baseColor, 1.0);
    }
}
