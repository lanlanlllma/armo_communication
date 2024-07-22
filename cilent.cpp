#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

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

int main() {
    // Create a socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    // Set up the server address and port
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8080);  // Replace with the actual server port
    if (inet_pton(AF_INET, "127.0.0.1", &(serverAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        return 1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        return 1;
    }

    // Send and receive messages in a loop
    while (true) {
        std::string sendMessage;
        std::cout << "Enter message to send: ";
        std::getline(std::cin, sendMessage);

        // Prepare the MessageBuffer
        MessageBuffer message{};
        message.Start = 0x0D00;
        message.MessageType = MessageType::STRING_MSG ; // Example type
        message.DataID = 1; // Example ID
        message.DataTotalLength = sendMessage.length();
        message.Offset = 0;
        message.DataLength = sendMessage.length();
        memcpy(message.Data, sendMessage.c_str(), sendMessage.length());
        message.End = 0x0721;

        // Serialize the message
        unsigned char buffer[10240];
        serializeMessage(message, buffer);

        // Send the message
        if (send(clientSocket, buffer, 20 + message.DataLength + 2, 0) < 0) {
            std::cerr << "Failed to send message." << std::endl;
            break;
        }

        // Receive a response from the server
        unsigned char recvBuffer[10240] = {0};
        int bytesReceived = recv(clientSocket, recvBuffer, sizeof(recvBuffer), 0);
        if (bytesReceived < 0) {
            std::cerr << "Failed to receive response." << std::endl;
            break;
        }

        // Deserialize the response
        MessageBuffer response;
        deserializeMessage(recvBuffer, response);

        std::cout << "Server response: ";
        std::cout.write(reinterpret_cast<char*>(response.Data), response.DataLength);
        std::cout << std::endl;

        // Optionally, add an exit condition to break the loop
        if (sendMessage == "exit") {
            break;
        }

        // Sleep for a while before sending the next message
        sleep(1);
    }

    // Close the socket
    close(clientSocket);

    return 0;
}
