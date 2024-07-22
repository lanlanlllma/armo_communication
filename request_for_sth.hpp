#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "frame_processor.hpp"

enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};
const unsigned short END_SYMBOL = 0x0721;

struct MessageBuffer {
    unsigned short Start;                     // 0x0D00
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int DataTotalLength;
    unsigned int Offset;
    unsigned int DataLength;
    unsigned char Data[10218];
    unsigned short End;                       // 0x0721
};

struct CameraInfoData {
    double CameraMatrix[9];
    double DistortionCoefficients[5];
};

struct TransformData {
    double Translation[3];
    double Rotation[4];
};

struct TransformRequestData {
    char From[10218 / 2];
    char To[10218 / 2];
};

void serializeMessage(const MessageBuffer& message, unsigned char* buffer) {
    memcpy(buffer, &message.Start, sizeof(message.Start));
    memcpy(buffer + 2, &message.MessageType, sizeof(message.MessageType));
    memcpy(buffer + 4, &message.DataID, sizeof(message.DataID));
    memcpy(buffer + 8, &message.DataTotalLength, sizeof(message.DataTotalLength));
    memcpy(buffer + 12, &message.Offset, sizeof(message.Offset));
    memcpy(buffer + 16, &message.DataLength, sizeof(message.DataLength));
    memcpy(buffer + 20, &message.Data, message.DataLength);
    memcpy(buffer + 20 + message.DataLength, &message.End, sizeof(message.End));
}

class Application {
public:
    unsigned char* receive_and_decode(MessageBuffer& message);
    void encode_and_send(unsigned char* buffer, int clientSocket, MessageBuffer &message) ;
        

    cv::Mat handle_image_msg(MessageBuffer& buffer, FrameProcessor& FrameProcessor,int cilentSocket);
    std::string handle_string_msg(MessageBuffer& buffer);
private:
    std::unordered_map<unsigned int, std::vector<unsigned char>> data_temp;
};

void deserializeMessage(unsigned char* buffer, MessageBuffer& message) {
    memcpy(&message.Start, buffer, sizeof(message.Start));
    memcpy(&message.MessageType, buffer + 2, sizeof(message.MessageType));
    memcpy(&message.DataID, buffer + 4, sizeof(message.DataID));
    memcpy(&message.DataTotalLength, buffer + 8, sizeof(message.DataTotalLength));
    memcpy(&message.Offset, buffer + 12, sizeof(message.Offset));
    memcpy(&message.DataLength, buffer + 16, sizeof(message.DataLength));
    memcpy(&message.Data, buffer + 20, message.DataLength);
    memcpy(&message.End, buffer + 20 + message.DataLength, sizeof(message.End));
}

unsigned char* Application::receive_and_decode(MessageBuffer& message) {
    unsigned int offset = message.Offset;
    unsigned int length = message.DataLength;
    unsigned int total_length = message.DataTotalLength;
    unsigned int dataID = message.DataID;

    if (data_temp.find(dataID) == data_temp.end()) {
        data_temp[dataID] = std::vector<unsigned char>(total_length);
    }

    std::memcpy(data_temp[dataID].data() + offset, message.Data, length);

    if (offset + length >= total_length) {
        unsigned char* data = new unsigned char[total_length];
        std::memcpy(data, data_temp[dataID].data(), total_length);
        data_temp.erase(dataID);
        return data;
    } else {
        return nullptr;
    }
}

void Application::encode_and_send(unsigned char* buffer, int clientSocket, MessageBuffer &message) {
    serializeMessage(message, buffer);
    ssize_t sendsat=send(clientSocket, buffer, sizeof(MessageBuffer), 0);
    if (sendsat == -1) {
        std::cerr << "Failed to send message to server." << std::endl;
        close(clientSocket);
    }
    // std::cout<<"Sent package."<<std::endl;

}



std::string Application::handle_string_msg(MessageBuffer& buffer) {
    return std::string(reinterpret_cast<char*>(buffer.Data), buffer.DataLength);
}

// request for camera info
void getCamInfo(sockaddr_in serverAddress,cv::Mat cameraMatrix, cv::Mat distCoeffs){
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return;
    }
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Failed to connect to server." << std::endl;
        close(clientSocket);
        return;
    }
    std::vector<unsigned char> completeMessageBuffer;
    while (true) {
        unsigned char recvBuffer[10240] = {0};
        // std::cout<<"Connected to server."<<std::endl;
        ssize_t bytesRead = recv(clientSocket, recvBuffer, sizeof(recvBuffer)-completeMessageBuffer.size(), 0);
        // std::cout<<"Recved package."<<std::endl;
        if (bytesRead == -1) {
            std::cerr << "Failed to receive message from server." << std::endl;
            close(clientSocket);
            break;
        }

        if (bytesRead == 0) {
            std::cout << "Server disconnected." << std::endl;
            close(clientSocket);
            break;
        }

        completeMessageBuffer.insert(completeMessageBuffer.end(), recvBuffer, recvBuffer + bytesRead);
        // std::cout<<"Received "<<bytesRead<<" bytes."<<std::endl;
        // std::cout<<"Buffer size: "<<completeMessageBuffer.size()<<std::endl;
        // std::cout<<"Message size: "<<sizeof(MessageBuffer)<<std::endl;

        while (completeMessageBuffer.size() >= sizeof(MessageBuffer)) {
            MessageBuffer receivedMessage;
            deserializeMessage(completeMessageBuffer.data(), receivedMessage);

            size_t messageSize = sizeof(MessageBuffer) - sizeof(receivedMessage.Data) + receivedMessage.DataLength;
            if (completeMessageBuffer.size() < messageSize) {
                break;
            }

            completeMessageBuffer.clear();
        if (receivedMessage.MessageType == CAMERA_INFO) {
            CameraInfoData cameraInfo;
            std::memcpy(&cameraInfo, receivedMessage.Data, receivedMessage.DataLength);
            std::memcpy(cameraMatrix.data, cameraInfo.CameraMatrix, sizeof(cameraInfo.CameraMatrix));
            std::memcpy(distCoeffs.data, cameraInfo.DistortionCoefficients, sizeof(cameraInfo.DistortionCoefficients));
            std::cout << "Received camera info." << std::endl;
            break;
        }
        }
        // put into mat
        cameraMatrix= cv::Mat(3, 3, CV_64F, cameraMatrix.data);
        distCoeffs= cv::Mat(1, 5, CV_64F, distCoeffs.data);
    }
}

// request for transform
void getTransform(sockaddr_in serverAddress, std::string from, std::string to, cv::Mat& translation, cv::Mat& rotation){
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return;
    }
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Failed to connect to server." << std::endl;
        close(clientSocket);
        return;
    }
    std::vector<unsigned char> completeMessageBuffer;
    while (true) {
        unsigned char recvBuffer[10240] = {0};
        // std::cout<<"Connected to server."<<std::endl;
        ssize_t bytesRead = recv(clientSocket, recvBuffer, sizeof(recvBuffer)-completeMessageBuffer.size(), 0);
        // std::cout<<"Recved package."<<std::endl;
        if (bytesRead == -1) {
            std::cerr << "Failed to receive message from server." << std::endl;
            close(clientSocket);
            break;
        }

        if (bytesRead == 0) {
            std::cout << "Server disconnected." << std::endl;
            close(clientSocket);
            break;
        }
        while(true){

        }
