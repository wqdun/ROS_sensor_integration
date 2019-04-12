// 3 6
// 10 15 12
// 1 9 12 23 26 37


#include <iostream>
#include <vector>

void GetBoundary(const std::vector<int> &numOfRooms, std::vector<int> &boundaries) {
    int sum = 0;
    for (int i = 0; i < numOfRooms.size(); ++i) {
        sum += numOfRooms[i];
        boundaries.push_back(sum);
    }
}




int GetBuildingId(int roomId, const std::vector<int> &boundaries) {
    if (roomId > boundaries.back()) {
        exit(1);
    }

    for (int i = 0; i < boundaries.size(); ++i) {
        if (roomId < boundaries[i]) {
            return (i + 1);
        }
    }
    return 0;
}

int main() {
    std::vector<int> numOfRooms {10, 15, 12};
    std::vector<int> boundaries;
    boundaries.clear();
    GetBoundary(numOfRooms, boundaries);

    for (auto i: boundaries) {
        std::cout << i << " ";
    }

    std::vector<int> roomId {37};
    for (int i = 0; i < roomId.size(); ++i) {
        int buildingId = GetBuildingId(roomId[i], boundaries);
        std::cout << roomId[i] << ": " << buildingId << "\n";
        int roomIdOfBuilding = -1;
        if (1 == buildingId) {
            roomIdOfBuilding = roomId[i];
        }
        roomIdOfBuilding = roomId[i] - boundaries[buildingId - 1];

        std::cout << roomId[i] << " is " <<  roomIdOfBuilding << " roomIdOfBuilding; " << roomIdOfBuilding  << " roomIdOfBuilding.\n";
    }

}


