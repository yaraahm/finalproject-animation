#version 330

in vec2 texCoord0;
in vec3 normal0;
in vec3 color0;
in vec3 position0;

uniform vec4 lightColor;
uniform sampler2D sampler1;
uniform vec4 lightDirection;

out vec4 Color;

void main()
{
	vec4 a = texture(sampler1, texCoord0);
	a.w = 0.5;
	Color = a; //you must have Color
}
