#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
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
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    // Bind the socket to a specific IP address and port
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(8080); // Change the port number if needed

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Failed to bind socket." << std::endl;
        close(serverSocket);
        return 1;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 5) == -1) {
        std::cerr << "Failed to listen for connections." << std::endl;
        close(serverSocket);
        return 1;
    }

    std::cout << "Server started. Listening for connections..." << std::endl;

    while (true) {
        // Accept a client connection
        sockaddr_in clientAddress{};
        socklen_t clientAddressLength = sizeof(clientAddress);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress, &clientAddressLength);
        if (clientSocket == -1) {
            std::cerr << "Failed to accept client connection." << std::endl;
            continue;
        }

        std::cout << "Client connected." << std::endl;

        while (true) {
            // Receive the message
            unsigned char recvBuffer[10240] = {0};
            ssize_t bytesRead = recv(clientSocket, recvBuffer, sizeof(recvBuffer), 0);
            if (bytesRead == -1) {
                std::cerr << "Failed to receive message from client." << std::endl;
                close(clientSocket);
                break;
            }

            if (bytesRead == 0) {
                std::cout << "Client disconnected." << std::endl;
                close(clientSocket);
                break;
            }

            MessageBuffer receivedMessage;
            deserializeMessage(recvBuffer, receivedMessage);

            std::cout << "Received message: " << receivedMessage.Data << std::endl;

            // Append "cailo" to the Data
            const char* appendString = "cailo";
            size_t appendLength = strlen(appendString);

            if (receivedMessage.DataLength + appendLength <= sizeof(receivedMessage.Data)) {
                memcpy(receivedMessage.Data + receivedMessage.DataLength, appendString, appendLength);
                receivedMessage.DataLength += appendLength;
                receivedMessage.DataTotalLength += appendLength;
            } else {
                std::cerr << "Not enough space to append string to Data field." << std::endl;
                close(clientSocket);
                break;
            }

            // Prepare the response
            receivedMessage.End = 0x0721;

            // Serialize the response
            unsigned char buffer[10240]={0};
            serializeMessage(receivedMessage, buffer);

            ssize_t bytesSent = send(clientSocket, buffer, 20 + receivedMessage.DataLength + 2, 0);
            if (bytesSent == -1) {
                std::cerr << "Failed to send message back to client." << std::endl;
                close(clientSocket);
                break;
            }

            std::cout << "Sent message back to client." << std::endl;
        }

        // Close the client socket
        close(clientSocket);
    }

    // Close the server socket
    close(serverSocket);

    return 0;
}
