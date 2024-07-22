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
        FrameProcessor.processFrame(img);
        std::cout<<"Saved "<<cnt<<std::endl;
        delete[] data;
        // send back summary
        MessageBuffer message;
        size_t firstPartSize = FrameProcessor.summary[0][FrameProcessor.fcount].size();
        size_t secondPartSize = FrameProcessor.summary[1][FrameProcessor.fcount].size();
        memcpy(message.Data, FrameProcessor.summary[0][FrameProcessor.fcount].data(), firstPartSize);
        memcpy(message.Data + firstPartSize, FrameProcessor.summary[1][FrameProcessor.fcount].data(), secondPartSize);
        message.Start = 0x0D00;
        message.MessageType = STRING_MSG;
        message.DataID = buffer.DataID;
        message.DataTotalLength = 0;
        message.Offset = 0;
        message.DataLength = 0;
        message.End = 0x0721;
        unsigned char buffer[sizeof(MessageBuffer)];
        encode_and_send(buffer,cilentSocket,message);
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

    // Set up the server address and port
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8000);  // Replace with the actual server port
    if (inet_pton(AF_INET, "10.2.20.66", &(serverAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        close(clientSocket);
        return 1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(clientSocket);
        return 1;
    }
    else{
        std::cout<<"Connected to server."<<std::endl;
    }

    Application app;
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
                    cv::waitKey(1);
                }
            } else if (receivedMessage.MessageType == STRING_MSG) {
                std::string str = app.handle_string_msg(receivedMessage);
                std::cout << "Received string message: " << str << std::endl;
            }
        }

    }
    // Close the socket
    close(clientSocket);

    return 0;
}
