#include <RobotManager.h>


int main() {
    uint16_t N;
    std::cin>>N;

    for (uint16_t i = 0; i < N; i++) {
        RobotManager::GetInstance().ReadCommandAndUse();
    }

    return 0;
}