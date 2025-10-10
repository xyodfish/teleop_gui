#pragma once
#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include "stb_image.h"

#include <vector>

#include <iostream>

namespace omnilink::gui {

    struct Vertex {
        glm::vec3 position_;
        glm::vec3 normal_;
        glm::vec2 texCoords_;
    };

    struct Texture {
        unsigned int id_;
        std::string type_;
    };

    struct Mesh {
        std::vector<Vertex> vertices_;
        std::vector<unsigned int> indices_;
        std::vector<Texture> textures_;
        glm::vec3 diffuseColor_ = glm::vec3(0.8f, 0.8f, 0.8f);
        unsigned int VAO_ = 0, VBO_ = 0, EBO_ = 0;

        void setup();
        void draw(GLuint shader);
    };

    class Model {
       public:
        std::vector<Mesh> meshes_;
        std::string directory_;

        void loadAssimp(const std::string& path);

       private:
        void processNode(aiNode* node, const aiScene* scene);

        Mesh processMesh(aiMesh* mesh, const aiScene* scene);

        unsigned int loadTexture(const std::string& path);
    };
}  // namespace omnilink::gui
