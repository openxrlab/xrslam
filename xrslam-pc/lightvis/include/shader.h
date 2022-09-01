#ifndef LIGHTVIS_SHADER_H
#define LIGHTVIS_SHADER_H

#include <Eigen/Eigen>
#include <glbinding/gl/gl.h>

namespace lightvis {

template <typename E> inline gl::GLboolean is_type_integral() {
    return gl::GL_FALSE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLbyte>() {
    return gl::GL_TRUE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLshort>() {
    return gl::GL_TRUE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLint>() {
    return gl::GL_TRUE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLubyte>() {
    return gl::GL_TRUE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLushort>() {
    return gl::GL_TRUE;
}

template <> inline gl::GLboolean is_type_integral<gl::GLuint>() {
    return gl::GL_TRUE;
}

template <typename E> inline gl::GLenum get_type_enum() {
    puts("Error getting type enum.");
    exit(0);
    return gl::GL_NONE;
}

template <> inline gl::GLenum get_type_enum<gl::GLbyte>() {
    return gl::GL_BYTE;
}

template <> inline gl::GLenum get_type_enum<gl::GLshort>() {
    return gl::GL_SHORT;
}

template <> inline gl::GLenum get_type_enum<gl::GLint>() { return gl::GL_INT; }

template <> inline gl::GLenum get_type_enum<gl::GLubyte>() {
    return gl::GL_UNSIGNED_BYTE;
}

template <> inline gl::GLenum get_type_enum<gl::GLushort>() {
    return gl::GL_UNSIGNED_SHORT;
}

template <> inline gl::GLenum get_type_enum<gl::GLuint>() {
    return gl::GL_UNSIGNED_INT;
}

template <> inline gl::GLenum get_type_enum<gl::GLfloat>() {
    return gl::GL_FLOAT;
}

class Shader {
  public:
    Shader(const char *vshader_source, const char *fshader_source) {
        gl::GLint status;

        vshader = gl::glCreateShader(gl::GL_VERTEX_SHADER);
        gl::glShaderSource(vshader, 1, &vshader_source, 0);
        gl::glCompileShader(vshader);
        gl::glGetShaderiv(vshader, gl::GL_COMPILE_STATUS, &status);
        if (status != (gl::GLint)gl::GL_TRUE) {
            puts("Error compiling vertex shader.");
            exit(0);
        }

        fshader = gl::glCreateShader(gl::GL_FRAGMENT_SHADER);
        gl::glShaderSource(fshader, 1, &fshader_source, 0);
        gl::glCompileShader(fshader);
        gl::glGetShaderiv(fshader, gl::GL_COMPILE_STATUS, &status);
        if (status != (gl::GLint)gl::GL_TRUE) {
            puts("Error compiling fragment shader.");
            exit(0);
        }

        program = gl::glCreateProgram();
        gl::glAttachShader(program, vshader);
        gl::glAttachShader(program, fshader);
        gl::glLinkProgram(program);
        gl::glGetProgramiv(program, gl::GL_LINK_STATUS, &status);
        if (status != (gl::GLint)gl::GL_TRUE) {
            puts("Error linking shader.");
            exit(0);
        }

        gl::glGenBuffers(1, &index_buffer);
        gl::glGenVertexArrays(1, &vertex_array);
    }

    ~Shader() {
        for (auto [attrib, buffer] : attribute_buffers) {
            gl::glDeleteBuffers(1, &buffer);
        }
        gl::glDeleteBuffers(1, &index_buffer);
        gl::glDeleteVertexArrays(1, &vertex_array);

        gl::glDetachShader(program, fshader);
        gl::glDetachShader(program, vshader);
        gl::glDeleteProgram(program);
        gl::glDeleteShader(fshader);
        gl::glDeleteShader(vshader);
    }

    void bind() {
        gl::glUseProgram(program);
        gl::glBindVertexArray(vertex_array);
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    }

    void unbind() {
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, 0);
        gl::glBindVertexArray(0);
        gl::glUseProgram(0);
    }

    void set_uniform(const std::string &name, const float &value) {
        gl::GLint uni = uniform(name);
        gl::glUniform1f(uni, value);
    }

    void set_uniform(const std::string &name, const Eigen::Vector3f &vector) {
        gl::GLint uni = uniform(name);
        gl::glUniform3fv(uni, 1, vector.data());
    }

    void set_uniform(const std::string &name, const Eigen::Vector4f &vector) {
        gl::GLint uni = uniform(name);
        gl::glUniform4fv(uni, 1, vector.data());
    }

    void set_uniform(const std::string &name, const Eigen::Matrix4f &matrix) {
        gl::GLint uni = uniform(name);
        gl::glUniformMatrix4fv(uni, 1, gl::GL_FALSE, matrix.data());
    }

    template <typename E, int N>
    void set_attribute(const std::string &name,
                       const std::vector<Eigen::Matrix<E, N, 1>> &data) {
        gl::GLint attrib = attribute(name);
        if (attribute_buffers.count(attrib) == 0) {
            gl::GLuint buffer;
            gl::glGenBuffers(1, &buffer);
            attribute_buffers[attrib] = buffer;
        }
        gl::GLuint buffer = attribute_buffers.at(attrib);
        gl::glBindBuffer(gl::GL_ARRAY_BUFFER, buffer);
        gl::glBufferData(gl::GL_ARRAY_BUFFER, sizeof(E) * N * data.size(),
                         &data[0], gl::GL_DYNAMIC_DRAW);
        gl::glEnableVertexAttribArray(attrib);
        gl::glVertexAttribPointer(attrib, N, get_type_enum<E>(),
                                  is_type_integral<E>(), 0, nullptr);
        gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
    }

    void set_indices(const std::vector<unsigned int> &indices) {
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, index_buffer);
        gl::glBufferData(gl::GL_ELEMENT_ARRAY_BUFFER,
                         sizeof(unsigned int) * indices.size(), &indices[0],
                         gl::GL_DYNAMIC_DRAW);
    }

    void draw(gl::GLenum mode, gl::GLuint start, gl::GLuint count) {
        gl::glDrawArrays(mode, start, count);
    }

    void draw_indexed(gl::GLenum mode, gl::GLuint start, gl::GLuint count) {
        gl::glDrawElements(mode, count, gl::GL_UNSIGNED_INT,
                           (const void *)(start * sizeof(gl::GLuint)));
    }

  private:
    gl::GLint uniform(const std::string &name) {
        if (uniforms.count(name) == 0) {
            gl::GLint location =
                gl::glGetUniformLocation(program, name.c_str());
            if (location == -1) {
                puts("Error getting uniform location.");
                exit(0);
            }
            uniforms[name] = location;
        }
        return uniforms.at(name);
    }

    gl::GLint attribute(const std::string &name) {
        if (attributes.count(name) == 0) {
            gl::GLint location = gl::glGetAttribLocation(program, name.c_str());
            if (location == -1) {
                puts("Error getting attribute location.");
                exit(0);
            }
            attributes[name] = location;
        }
        return attributes.at(name);
    }

    gl::GLuint program;
    gl::GLuint vshader;
    gl::GLuint fshader;
    std::map<std::string, gl::GLint> uniforms;
    std::map<std::string, gl::GLint> attributes;
    std::map<gl::GLint, gl::GLuint> attribute_buffers;
    gl::GLuint index_buffer;
    gl::GLuint vertex_array;
};

} // namespace lightvis

#endif // LIGHTVIS_SHADER_H
