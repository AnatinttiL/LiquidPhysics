#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define GLFW_INCLUDE_NONE
#include "include/glad/gl.h"
#include "include/GLFW/glfw3.h"


void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

class Graphics {
    private:
       GLFWwindow* window; 

    public:
        Graphics() {
            if (!glfwInit())
                throw "Failed GLFW Initialization!";
            glfwSetErrorCallback(error_callback);

            window = glfwCreateWindow(640, 480, "Physics", NULL, NULL);
            if (!window) {
                throw "Failed GLFW Window creation!";
            }

            glfwMakeContextCurrent(window);
            gladLoadGL(glfwGetProcAddress);
        }

        void Draw() {

            while (!glfwWindowShouldClose(window))
            {
                // Keep running
            }
        }

        void End() {
            glfwDestroyWindow(window);
            glfwTerminate();
        }

        
};

int main(){
    Graphics graphics = Graphics();

    graphics.Draw();
    graphics.End();
}