#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

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
            close(serverSocket);
            return 1;
        }

        std::cout << "Client connected." << std::endl;

        while(true){
        // Receive and echo back the message
        char buffer[10240]={0};
        ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesRead == -1) {
            std::cerr << "Failed to receive message from client." << std::endl;
            close(clientSocket);
            close(serverSocket);
            return 1;
        }

        if (bytesRead == 0) {
            std::cout << "Client disconnected." << std::endl;
            close(clientSocket);
            continue;
        }

        std::cout << "Received message: " << buffer << std::endl;

        ssize_t bytesSent = send(clientSocket, buffer, bytesRead, 0);
        if (bytesSent == -1) {
            std::cerr << "Failed to send message back to client." << std::endl;
            close(clientSocket);
            close(serverSocket);
            return 1;
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