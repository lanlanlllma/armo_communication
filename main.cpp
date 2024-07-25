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
#include "request_for_sth.hpp"


cv::Mat Application::handle_image_msg(MessageBuffer& buffer, FrameProcessor& FrameProcessor,int cilentSocket) {
    unsigned char* data = receive_and_decode(buffer);
    if (data != nullptr) {
        std::vector<unsigned char> img_data(data, data + buffer.DataTotalLength);
        cv::Mat img = cv::imdecode(img_data, cv::IMREAD_COLOR);
        std::string cnt=std::to_string(buffer.DataID);
        cnt.append(".jpg");
        // cv::imwrite(cnt,img);
        std::string path;
        path="./";
        path.append(cnt);
        // cv::imwrite(path,img);
        cv::imshow("Image", img);
        FrameProcessor.processFrame(img);
        std::cout<<"Saved "<<cnt<<std::endl;
        delete[] data;
        // Get processing result
        return img;
    }
    return cv::Mat();
}

int main() {
    FrameProcessor processor;
    // Create a socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }
    int camclientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (camclientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }
    int transformclientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (transformclientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }
    int reportclientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (reportclientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    Application app;
    // Set up the server address and port
    sockaddr_in serverAddress{};
    sockaddr_in camseverAddress{};
    sockaddr_in transformAddress{};
    sockaddr_in reportAddress{};
    const char* serverIP = "10.2.20.66";
    camseverAddress.sin_family = AF_INET;
    transformAddress.sin_family = AF_INET;
    reportAddress.sin_family = AF_INET;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8000);  // Replace with the actual server port
    camseverAddress.sin_port = htons(5140);
    transformAddress.sin_port = htons(4399);
    reportAddress.sin_port = htons(8003);
    if (inet_pton(AF_INET, serverIP, &(serverAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        close(clientSocket);
        return 1;
    }
    if (inet_pton(AF_INET, serverIP, &(camseverAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        close(camclientSocket);
        return 1;
    }
    if (inet_pton(AF_INET, serverIP, &(transformAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        close(transformclientSocket);
        return 1;
    }
    // if (inet_pton(AF_INET, serverIP, &(reportAddress.sin_addr)) <= 0) {
    //     std::cerr << "Invalid address/Address not supported." << std::endl;
    //     close(reportclientSocket);
    //     return 1;
    // }
    // connect to caminfo
    if (connect(camclientSocket, (struct sockaddr*)&camseverAddress, sizeof(camseverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(camclientSocket);
        return 1;
    }
    else{
        std::cout<<"Connected to caminfo."<<std::endl;
    }
    getCamInfo(camclientSocket,processor);
    // show result
    std::cout<<"cameramatrix: "<<processor.cameraMatrix<<std::endl;
    std::cout<<"distCoeffs: "<<processor.distCoeffs<<std::endl;
    close(camclientSocket);

    // connect to transform
    if (connect(transformclientSocket, (struct sockaddr*)&transformAddress, sizeof(transformAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(transformclientSocket);
        return 1;
    }
    else{
        std::cout<<"Connected to transform."<<std::endl;
    }
    double camtranslation[3];
    double camrotation[3];
    double gimtranslation[3];
    double gimrotation[3];
    getTransform(transformclientSocket,"Odom","Gimbal",gimtranslation,gimrotation,app);
    getTransform(transformclientSocket,"Gimbal","Camera",camtranslation,camrotation,app);

    close(transformclientSocket);
    processor.setTransform(camtranslation,camrotation,gimtranslation,gimrotation);
    // show result
    std::cout<<"camtranslation: "<<camtranslation[0]<<" "<<camtranslation[1]<<" "<<camtranslation[2]<<std::endl;
    std::cout<<"camrotation: "<<processor.transform_array.roll_camera<<" "<<processor.transform_array.yaw_camera<<" "<<processor.transform_array.pitch_gimbal<<std::endl;
    // std::cout<<"gimtranslation: "<<gimtranslation[0]<<" "<<gimtranslation[1]<<" "<<gimtranslation[2]<<std::endl;
    // std::cout<<"gimrotation: "<<gimrotation[0]<<" "<<gimrotation[1]<<" "<<gimrotation[2]<<std::endl;
    // // connect to report
    // if (connect(reportclientSocket, (struct sockaddr*)&reportAddress, sizeof(reportAddress)) < 0) {
    //     std::cerr << "Connection failed." << std::endl;
    //     close(reportclientSocket);
    //     return 1;
    // }
    // else{
    //     std::cout<<"Connected to report."<<std::endl;
    // }
    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(clientSocket);
        return 1;
    }
    else{
        std::cout<<"Connected to server."<<std::endl;
    }

    std::vector<unsigned char> completeMessageBuffer;
    // Send and receive messages in a loop
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
            

            if (receivedMessage.MessageType == IMAGE_MSG) {
                // std::cout<<"recieved image message."<<std::endl;
                cv::Mat img = app.handle_image_msg(receivedMessage, processor,clientSocket);
                if (!img.empty()) {
                    // cv::imshow("Image", img);
                    reportsummary(clientSocket,processor);
                    cv::waitKey(1);
                }
            } 
            else if (receivedMessage.MessageType == STRING_MSG) {
                std::string str = app.handle_string_msg(receivedMessage);
                if(!str.empty()){
                    std::cout << "Received string message: " << str << std::endl;
                }
            }
        }
    }
    // Close the socket
    close(clientSocket);

    return 0;
}
