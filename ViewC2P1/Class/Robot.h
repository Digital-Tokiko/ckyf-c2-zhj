//
// Created by mizu on 2026/1/31.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <cstdint>

enum {
    kInfantry = 0,
    kEngineer
};

class Robot {
private:
    uint16_t max_health_;
    uint16_t max_heat_;
    uint16_t health_;
    uint16_t heat_;
    uint16_t team_id_;
    uint16_t id_;

    uint8_t type_;
    uint8_t level_;

public:
    Robot() = default;

    ~Robot() = default;

    void SetMaxHealth(const uint16_t max_health) {
        max_health_ = max_health;
        SetHealth(max_health);
    }

    void SetMaxHeat(const uint16_t max_heat) { max_heat_ = max_heat; }
    void SetHealth(const uint16_t health) { health_ = health; }
    void SetHeat(const uint16_t heat) { heat_ = heat; }
    void SetTeamId(const uint16_t team_id) { team_id_ = team_id; }
    void SetId(const uint16_t id) { id_ = id; }
    void SetType(const uint8_t type) { type_ = type; }
    void SetLevel(const uint8_t level) { level_ = level; }

    [[nodiscard]] uint16_t GetMaxHealth() const { return max_health_; }
    [[nodiscard]] uint16_t GetMaxHeat() const { return max_heat_; }
    [[nodiscard]] uint16_t GetHealth() const { return health_; }
    [[nodiscard]] uint16_t GetHeat() const { return heat_; }
    [[nodiscard]] uint16_t GetTeamId() const { return team_id_; }
    [[nodiscard]] uint16_t GetId() const { return id_; }
    [[nodiscard]] uint8_t GetType() const { return type_; }
    [[nodiscard]] uint8_t GetLevel() const { return level_; }

    [[nodiscard]] bool IsDestroyed() const {
        if (health_ <= max_health_ && health_ > 0) return false;
        else return true;
    }

    void Upgrade(uint8_t to_level);

    void TakeDamage(const uint16_t damage) { SetHealth(GetHealth() - damage); }

    void TakeDamageForHeat(uint16_t heat_change);
};


#endif //ROBOT_H
