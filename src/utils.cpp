#include "utils.hpp"

DisjointSetUnion::DisjointSetUnion(int n) : n_(n) {
    leader_.resize(n);
    std::iota(leader_.begin(), leader_.end(), 0);
}

bool DisjointSetUnion::Merge(int x, int y) {
    x = FindLeader(x);
    y = FindLeader(y);

    if (x == y) {
        return false;
    }

    leader_[y] = x;
    return true;
}

int DisjointSetUnion::FindLeader(int x) {
    return leader_[x] == x ? x : (leader_[x] = FindLeader(leader_[x]));
}

std::vector<std::vector<int>> DisjointSetUnion::GetSets() {
    std::vector<std::vector<int>> sets(n_);
    for (int i = 0; i < n_; i++) {
        sets[FindLeader(i)].push_back(i);
    }
    sets.erase(
        std::remove_if(sets.begin(), sets.end(), [](const std::vector<int> vec) { return vec.empty(); }),
        sets.end());

    for (auto &vec : sets) {
        std::sort(vec.begin(), vec.end());
    }
    return sets;
}

std::ostream &operator<<(std::ostream &os, const Position &pos) {
    os << "(" << pos[0] << ", " << pos[1] << ")";
    return os;
}

Position operator+(const Position &lhs, const Position &rhs) {
    return Position{lhs[0] + rhs[0], lhs[1] + rhs[1]};
}

Position operator-(const Position &lhs, const Position &rhs) {
    return Position{lhs[0] - rhs[0], lhs[1] - rhs[1]};
}

int GetL1(const Position &lhs, const Position &rhs) {
    return std::abs(lhs[0] - rhs[0]) + std::abs(lhs[1] - rhs[1]);
}

Logger crashLog("crash");