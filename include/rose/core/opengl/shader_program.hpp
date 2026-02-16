//
// Created by orange on 16.02.2026.
//
#pragma once
#include <GL/glew.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
namespace rose::core::opengl
{
    class ShaderError final : public std::runtime_error
    {
    public:
        explicit ShaderError(const std::string& msg): std::runtime_error(msg)
        {
        }
    };

    class ShaderProgram final
    {
    public:
        ShaderProgram() = default;

        ShaderProgram(std::string_view vertexSrc, std::string_view fragmentSrc, std::string_view geometrySrc = {})
        {
            create(vertexSrc, fragmentSrc, geometrySrc);
        }

        // Factory: build from files.
        static ShaderProgram from_files(
                const std::filesystem::path& vertex_path,
                const std::filesystem::path& fragment_path,
                const std::filesystem::path& geometry_path = {}
        )
        {
            const std::string v = read_text_file(vertex_path);
            const std::string f = read_text_file(fragment_path);

            if (geometry_path.empty())
            {
                return {v, f};
            }

            const std::string g = read_text_file(geometry_path);
            return {v, f, g};
        }

        ~ShaderProgram()
        {
            destroy();
        }

        ShaderProgram(const ShaderProgram&) = delete;
        ShaderProgram& operator=(const ShaderProgram&) = delete;

        ShaderProgram(ShaderProgram&& other) noexcept
            : m_program(std::exchange(other.m_program, 0)), uniformCache_(std::move(other.uniformCache_))
        {
        }

        ShaderProgram& operator=(ShaderProgram&& other) noexcept
        {
            if (this != &other)
            {
                destroy();
                m_program = std::exchange(other.m_program, 0);
                uniformCache_ = std::move(other.uniformCache_);
            }
            return *this;
        }

        bool valid() const noexcept
        {
            return m_program != 0;
        }
        explicit operator bool() const noexcept
        {
            return valid();
        }

        GLuint id() const noexcept
        {
            return m_program;
        }

        // Rebuild shader program from source strings.
        void create(const std::string_view vertex_src, const std::string_view fragment_src,
               const std::string_view geometry_src = {})
        {
            destroy();

            std::vector<GLuint> shader_ids;
            shader_ids.reserve(3);

            try
            {
                shader_ids.push_back(compile_shader(GL_VERTEX_SHADER, vertex_src));
                shader_ids.push_back(compile_shader(GL_FRAGMENT_SHADER, fragment_src));

                if (!geometry_src.empty())
                {
                    shader_ids.push_back(compile_shader(GL_GEOMETRY_SHADER, geometry_src));
                }

                GLuint prog = glCreateProgram();
                if (prog == 0)
                {
                    throw ShaderError("glCreateProgram() failed.");
                }

                for (GLuint s : shader_ids)
                {
                    glAttachShader(prog, s);
                }

                glLinkProgram(prog);

                GLint linked = 0;
                glGetProgramiv(prog, GL_LINK_STATUS, &linked);
                if (!linked)
                {
                    const std::string log = get_program_log(prog);
                    glDeleteProgram(prog);
                    throw ShaderError("Program link failed:\n" + log);
                }

                for (GLuint s : shader_ids)
                {
                    glDetachShader(prog, s);
                    glDeleteShader(s);
                }

                m_program = prog;
                uniformCache_.clear();
            }
            catch (...)
            {
                for (GLuint s : shader_ids)
                {
                    glDeleteShader(s);
                }
                throw;
            }
        }

        // Bind for drawing / classic uniform setting.
        void use() const
        {
            if (!m_program)
            {
                throw ShaderError("Attempted to use an invalid shader program.");
            }
            glUseProgram(m_program);
        }

        // Uniform helpers (requires this program to be active via use()).
        void set_bool(const std::string_view name, const bool value) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform1i(loc, value ? 1 : 0);
        }

        void set_int(std::string_view name, int value) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform1i(loc, value);
        }

        void set_float(std::string_view name, float value) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform1f(loc, value);
        }

        void set_vec2(std::string_view name, float x, float y) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform2f(loc, x, y);
        }

        void set_vec3(std::string_view name, float x, float y, float z) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform3f(loc, x, y, z);
        }

        void set_vec4(std::string_view name, float x, float y, float z, float w) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
                glUniform4f(loc, x, y, z, w);
        }

        // value points to 16 floats (column-major by default, as OpenGL expects).
        void set_mat4(const std::string_view name, const float* value, const bool transpose = false) const
        {
            const GLint loc = uniform_location(name);
            if (loc >= 0)
            {
                glUniformMatrix4fv(loc, 1, transpose ? GL_TRUE : GL_FALSE, value);
            }
        }

    private:
        GLuint m_program = 0;
        mutable std::unordered_map<std::string, GLint> uniformCache_{};

        void destroy() noexcept
        {
            if (m_program != 0)
            {
                glDeleteProgram(m_program);
                m_program = 0;
            }
            uniformCache_.clear();
        }

        static std::string read_text_file(const std::filesystem::path& path)
        {
            std::ifstream file(path, std::ios::in | std::ios::binary);
            if (!file)
            {
                throw ShaderError("Failed to open shader file: " + path.string());
            }

            std::ostringstream ss;
            ss << file.rdbuf();
            return ss.str();
        }

        static GLuint compile_shader(GLenum type, std::string_view source)
        {
            GLuint shader = glCreateShader(type);
            if (shader == 0)
            {
                throw ShaderError("glCreateShader() failed for type " + shader_type_name(type));
            }

            const GLchar* src = source.data();
            const GLint len = static_cast<GLint>(source.size());
            glShaderSource(shader, 1, &src, &len);
            glCompileShader(shader);

            GLint compiled = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
            if (!compiled)
            {
                const std::string log = get_shader_log(shader);
                glDeleteShader(shader);
                throw ShaderError("Compile failed for " + shader_type_name(type) + " shader:\n" + log);
            }

            return shader;
        }

        GLint uniform_location(std::string_view name) const
        {
            std::string key(name);
            const auto it = uniformCache_.find(key);
            if (it != uniformCache_.end())
            {
                return it->second;
            }

            const GLint loc = glGetUniformLocation(m_program, key.c_str());
            uniformCache_.emplace(std::move(key), loc);
            return loc;
        }

        static std::string get_shader_log(GLuint shader)
        {
            GLint length = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);

            if (length <= 1)
                return {};

            std::string log(static_cast<size_t>(length), '\0');
            glGetShaderInfoLog(shader, length, nullptr, log.data());
            return log;
        }

        static std::string get_program_log(GLuint program)
        {
            GLint length = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);

            if (length <= 1)
                return {};

            std::string log(static_cast<size_t>(length), '\0');
            glGetProgramInfoLog(program, length, nullptr, log.data());
            return log;
        }

        static std::string shader_type_name(GLenum type)
        {
            switch (type)
            {
            case GL_VERTEX_SHADER:
                return "vertex";
            case GL_FRAGMENT_SHADER:
                return "fragment";
            case GL_GEOMETRY_SHADER:
                return "geometry";
            default:
                return "unknown";
            }
        }
    };
} // namespace rose::core::opengl