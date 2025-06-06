#ifndef __GLSHADER_H__  
#define __GLSHADER_H__  

#pragma GCC diagnostic ignored "-Wunused-variable"

#include <GL/glew.h>  
#include <GLFW/glfw3.h>

static const GLchar *grid_vshader = R"(
    #version 330
    uniform vec3 Location;
    uniform mat4 ProjMat;
    in vec4 Color;
    in vec3 Position;
    out vec3 Frag_Position;
    out vec4 Frag_Color;
    out vec3 Frag_Location;
    void main() {
        Frag_Position = Position;
        Frag_Color = Color;
        gl_Position = ProjMat * vec4(Position, 1);
        Frag_Location = Location;
    }
)";

static const GLchar *grid_fshader = R"(
    #version 330
    in vec3 Frag_Location;
    in vec3 Frag_Position;
    in vec4 Frag_Color;
    out vec4 Out_Color;
    void main(){
        float distance = length(Frag_Position - Frag_Location);
        float transparency = Frag_Color.a * (1.0- smoothstep(10, 100, distance));
        Out_Color = vec4(Frag_Color.r, Frag_Color.g, Frag_Color.b, transparency);
    }
)";

static const GLchar *position_vshader = R"(
    #version 400
    uniform mat4 ProjMat;
    uniform vec3 Location;
    uniform float Scale;
    in vec3 Position;
    in vec4 Color;
    out vec3 Frag_Position;
    out vec4 Frag_Color;
    void main() {
        vec3 p = Scale * (Position - Location);
        Frag_Position = p;
        Frag_Color = Color;
        gl_Position = ProjMat * vec4(p, 1);
        // gl_PointSize = 100; // TODO: linewith cannot work
    }
)";

static const GLchar *position_fshader = R"(
    #version 400
    in vec3 Frag_Position;
    in vec4 Frag_Color;
    out vec4 Out_Color;
    void main(){
        vec3 r = 1.0 - smoothstep(10, 100, abs(Frag_Position));
        vec4 c = vec4(Frag_Color.rgb, Frag_Color.a * min(min(r.x, r.y), r.z));
        Out_Color = c;
    }
)";

static const GLchar *points_vshader = R"(
    #version 400
    uniform mat4 ProjMat;
    uniform vec3 Location;
    uniform float Scale;
    uniform float PointSize;
    in vec3 Position;
    in vec4 Color;
    out vec3 Frag_Position;
    out vec4 Frag_Color;
    void main() {
        vec3 p = Scale * (Position - Location);
        Frag_Position = p;
        Frag_Color = Color;
        gl_Position = ProjMat * vec4(p, 1);
        gl_PointSize = PointSize;
    }
)";

static const GLchar *points_fshader = R"(
    #version 400
    in vec3 Frag_Position;
    in vec4 Frag_Color;
    out vec4 Out_Color;
    void main(){
        vec3 r = 1.0 - smoothstep(9.5, 10.5, abs(Frag_Position));
        vec4 c = vec4(Frag_Color.rgb, Frag_Color.a * min(min(r.x, r.y), r.z));
        Out_Color = c;
    }
)";


static const GLchar *texture_vshader = R"(
    #version 330
    uniform mat4 ProjMat;
    uniform vec3 Location;
    uniform float Scale;
    in vec2 TexCoords;
    in vec3 Position;
    out vec2 Frag_TexCoords;
    out vec3 Frag_Position;
    void main() {
        vec3 p = Scale * (Position - Location);
        Frag_Position = p;
        Frag_TexCoords = TexCoords;
        gl_Position = ProjMat * vec4(p, 1);
    }
)";

static const GLchar *texture_fshader = R"(
    #version 330
    uniform sampler2D Texture;
    in vec2 Frag_TexCoords;
    in vec3 Frag_Position;
    out vec4 Out_Color;
    void main(){
        vec2 texCoord = vec2(1.0 - Frag_TexCoords.y, 1.0 - Frag_TexCoords.x);
        Out_Color = texture(Texture, texCoord); 
    }
)";

static const GLchar *lighting_vshader = R"(
    #version 400
    uniform mat4 ProjMat;
    uniform vec3 Location;
    uniform float Scale;
    in vec3 Position;
    in vec3 Normal;
    in vec4 Color;
    out vec3 Frag_Position;
    out vec4 Frag_Color;
    out vec3 Frag_Normal;
    void main() {
        vec3 p = Scale * (Position - Location);
        Frag_Position = p;
        Frag_Color = Color;
        Frag_Normal = Normal;
        gl_Position = ProjMat * vec4(p, 1);
        // gl_PointSize = 100; // TODO: linewith cannot work
    }
)";

static const GLchar *lighting_fshader = R"(
    #version 400
    in vec3 Frag_Position;
    in vec4 Frag_Color;
    in vec3 Frag_Normal;
    out vec4 Out_Color;

    const vec3 ambientLightColor = vec3(0.2, 0.2, 0.2);
    vec3 lightPosition = vec3(0.0, 10.0, 10.0);
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    
    void main(){  
        vec3 norm = normalize(Frag_Normal);  
        vec3 lightDir = normalize(lightPosition - Frag_Position);  
        float diff = max(dot(norm, lightDir), 0.0);  
        vec3 ambient = ambientLightColor * Frag_Color.rgb;
        vec3 diffuse = lightColor * diff * Frag_Color.rgb; 
        vec3 result = ambient + diffuse;  
        Out_Color = vec4(result, Frag_Color.a);  
    }  
)";

#endif