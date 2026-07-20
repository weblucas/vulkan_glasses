#version 450

layout (binding = 0) uniform sampler2D texSampler;

layout (location = 0) in vec2 fragTexCoord;

layout (location = 0) out vec4 outFragColor;

layout(push_constant)
uniform PushConsts {
    layout( offset = 64 ) float idobj;
} pushConsts;

void main() 
{
    outFragColor = texture(texSampler, fragTexCoord).zyxw; 
    outFragColor.w = pushConsts.idobj;
}
