#ifndef __VIEWER_SHADER_H__
#define __VIEWER_SHADER_H__

#include <Eigen/Eigen>


template <typename E> inline GLenum is_type_integral() {
    return GL_FALSE;
}

template <> inline GLenum is_type_integral<GLbyte>() {
    return GL_TRUE;
}

template <> inline GLenum is_type_integral<GLshort>() {
    return GL_TRUE;
}

template <> inline GLenum is_type_integral<GLint>() {
    return GL_TRUE;
}

template <> inline GLenum is_type_integral<GLubyte>() {
    return GL_TRUE;
}

template <> inline GLenum is_type_integral<GLushort>() {
    return GL_TRUE;
}

template <> inline GLenum is_type_integral<GLuint>() {
    return GL_TRUE;
}

template <typename E> inline GLenum get_type_enum() {
    puts("Error getting type enum.");
    exit(0);
    return GL_NONE;
}

template <> inline GLenum get_type_enum<GLbyte>() {
    return GL_BYTE;
}

template <> inline GLenum get_type_enum<GLshort>() {
    return GL_SHORT;
}

template <> inline GLenum get_type_enum<GLint>() {
    return GL_INT;
}

template <> inline GLenum get_type_enum<GLubyte>() {
    return GL_UNSIGNED_BYTE;
}

template <> inline GLenum get_type_enum<GLushort>() {
    return GL_UNSIGNED_SHORT;
}

template <> inline GLenum get_type_enum<GLuint>() {
    return GL_UNSIGNED_INT;
}

template <> inline GLenum get_type_enum<GLfloat>() {
    return GL_FLOAT;
}


class Shader {
public:
    Shader(const char *vshader_source, const char *fshader_source) {
        GLint status;

        vshader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vshader, 1, &vshader_source, 0);
        glCompileShader(vshader);
        glGetShaderiv(vshader, GL_COMPILE_STATUS, &status);
        if (status != (GLint)GL_TRUE) {
            puts("Error compiling vertex shader.");
            exit(0);
        }

        fshader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fshader, 1, &fshader_source, 0);
        glCompileShader(fshader);
        glGetShaderiv(fshader, GL_COMPILE_STATUS, &status);
        if (status != (GLint)GL_TRUE) {
            puts("Error compiling fragment shader.");
            exit(0);
        }

        program = glCreateProgram();
        glAttachShader(program, vshader);
        glAttachShader(program, fshader);
        glLinkProgram(program);
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status != (GLint)GL_TRUE) {
            puts("Error linking shader.");
            exit(0);
        }

        glGenBuffers(1, &index_buffer);
        glGenVertexArrays(1, &vertex_array);
    }

    ~Shader() {
        for (auto [attrib, buffer] : attribute_buffers) {
            glDeleteBuffers(1, &buffer);
        }
        glDeleteBuffers(1, &index_buffer);
        glDeleteVertexArrays(1, &vertex_array);

        glDetachShader(program, fshader);
        glDetachShader(program, vshader);
        glDeleteProgram(program);
        glDeleteShader(fshader);
        glDeleteShader(vshader);
    }

    void bind() {
        glUseProgram(program);
        glBindVertexArray(vertex_array);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    }

    void unbind() {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
    }

    void set_uniform(const std::string &name, const float &value) {
        GLint uni = uniform(name);
        glUniform1f(uni, value);
    }

    void set_uniform(const std::string &name, const Eigen::Vector3f &vector) {
        GLint uni = uniform(name);
        glUniform3fv(uni, 1, vector.data());
    }

    void set_uniform(const std::string &name, const Eigen::Vector4f &vector) {
        GLint uni = uniform(name);
        glUniform4fv(uni, 1, vector.data());
    }

    void set_uniform(const std::string &name, const Eigen::Matrix4f &matrix) {
        GLint uni = uniform(name);
        glUniformMatrix4fv(uni, 1, GL_FALSE, matrix.data());
    }

    // texture
    void set_uniform(const std::string &name) {
        GLint uni = uniform(name);
        glUniform1i(uni, 0);
    }

    template <typename E, int N>
    void set_attribute(const std::string &name,
                    const std::vector<Eigen::Matrix<E, N, 1>> &data) {
        GLint attrib = attribute(name);
        if (attribute_buffers.count(attrib) == 0) {
            GLuint buffer;
            glGenBuffers(1, &buffer);
            attribute_buffers[attrib] = buffer;
        }
        GLuint buffer = attribute_buffers.at(attrib);
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(E) * N * data.size(),
                    &data[0], GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(attrib);
        glVertexAttribPointer(attrib, N, get_type_enum<E>(),
                            is_type_integral<E>(), 0, nullptr);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void set_indices(const std::vector<unsigned int> &indices) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    sizeof(unsigned int) * indices.size(), &indices[0],
                    GL_DYNAMIC_DRAW);
    }

    void draw(GLenum mode, GLuint start, GLuint count) {
        glDrawArrays(mode, start, count);
    }

    void draw_indexed(GLenum mode, GLuint start, GLuint count) {
        glDrawElements(mode, count, GL_UNSIGNED_INT,
                    (const void *)(start * sizeof(GLuint)));
    }

    private:
    GLint uniform(const std::string &name) {
        if (uniforms.count(name) == 0) {
            GLint location =
                glGetUniformLocation(program, name.c_str());
            if (location == -1) {
                puts("Error getting uniform location.");
                exit(0);
            }
            uniforms[name] = location;
        }
        return uniforms.at(name);
    }

    GLint attribute(const std::string &name) {
        if (attributes.count(name) == 0) {
            GLint location = glGetAttribLocation(program, name.c_str());
            if (location == -1) {
                puts("Error getting attribute location.");
                exit(0);
            }
            attributes[name] = location;
        }
        return attributes.at(name);
    }

    GLuint program;
    GLuint vshader;
    GLuint fshader;
    std::map<std::string, GLint> uniforms;
    std::map<std::string, GLint> attributes;
    std::map<GLint, GLuint> attribute_buffers;
    GLuint index_buffer;
    GLuint vertex_array;

};


#endif // LIGHTVIS_SHADER_H
