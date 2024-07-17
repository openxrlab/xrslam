#ifndef __FRAMEBUFFER_H__  
#define __FRAMEBUFFER_H__  

#include <GL/glew.h>  
#include <GLFW/glfw3.h>

class FrameBuffer{

private:
	GLuint fbo;
	GLuint texture;
	GLuint rbo;

public:

    FrameBuffer(){}

	FrameBuffer(const int width, const int height){

        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

        glGenRenderbuffers(1, &rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
    }

    void setup(const cv::Mat& image, int flag=0)
    {
        cv::Mat textureImage;
        if(image.channels() == 1)
            cv::cvtColor(image, textureImage, cv::COLOR_GRAY2RGB);
        else
            textureImage = image.clone();

        if(flag == 0){
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImage.cols, textureImage.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, textureImage.data);
            glGenerateMipmap(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, 0);
        }else{
            cv::flip(textureImage, textureImage, 0);
            glBindFramebuffer(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImage.cols, textureImage.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, textureImage.data);
            glGenerateMipmap(GL_TEXTURE_2D);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);   
            glBindFramebuffer(GL_FRAMEBUFFER, texture);
            glBindFramebuffer(GL_FRAMEBUFFER, 0); 
        }

    }

    ~FrameBuffer(){
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &texture);
        glDeleteRenderbuffers(1, &rbo);
    }

    GLuint getFrameTexture(){
        return texture;
    }

    void rescaleFrameBuffer(float width, float height){
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    }

    void bind() const{
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    }

    void unbind() const{
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


};



#endif