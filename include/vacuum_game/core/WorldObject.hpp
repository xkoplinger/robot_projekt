#ifndef WORLD_OBJECT_HPP
#define WORLD_OBJECT_HPP

#include <string>
#include <stdexcept>

// 1. Vlastná výnimka (podmienka zadania)
class VacuumException : public std::runtime_error {
public:
    explicit VacuumException(const std::string& msg) : std::runtime_error("Vacuum Error: " + msg) {}
};

// 2. Základná trieda
class WorldObject {
public:
    double x, y;
    int version = 0;
    virtual ~WorldObject() = default;
    virtual std::string getType() const = 0;
    
    // Zmenili sme "= 0" na predvolenú implementáciu, aby trieda Trash nebola abstraktná
    virtual std::string getColor() const { return "white"; } 
};

// 3. Odpadky
class Trash : public WorldObject
{
public:
    std::string color;
    double radius;

    Trash(
        double x_,
        double y_,
        const std::string& c,
        double r)
    {
        x = x_;
        y = y_;
        color = c;
        radius = r;
    }

    std::string getType() const override
    {
        return "trash";
    }

    std::string getColor() const override
    {
        return color;
    }
};

class PlasticTrash : public Trash {
public:
    PlasticTrash(double x_, double y_, double r)
        : Trash(x_, y_, "yellow", r)
    {}

    std::string getColor() const override {
        return "yellow";
    }
};

class PaperTrash : public Trash {
public:
    PaperTrash(double x_, double y_, double r)
        : Trash(x_, y_, "blue", r)
    {}

    std::string getColor() const override {
        return "blue";
    }
};

class GlassTrash : public Trash {
public:
    GlassTrash(double x_, double y_, double r)
        : Trash(x_, y_, "green", r)
    {}

    std::string getColor() const override {
        return "green";
    }
};

// 5. Prekážka
class Obstacle : public WorldObject {
public:
    double radius = 0.5;
    std::string getType() const override { return "obstacle"; }
    std::string getColor() const override { return "red"; }
};

#endif
