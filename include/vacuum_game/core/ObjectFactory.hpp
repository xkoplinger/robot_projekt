#pragma once
#include "WorldObject.hpp"
#include <memory>
#include <cstdlib>

class ObjectFactory {
public:
  static std::unique_ptr<WorldObject>
  createObject(
      const std::string& type,
      double x,
      double y,
      double radius = 0.3)
  {
      if (type == "trash")
      {
          int r = std::rand() % 3;

          if (r == 0)
          {
              return std::make_unique<PlasticTrash>(
                  x, y, radius);
          }
          else if (r == 1)
          {
              return std::make_unique<PaperTrash>(
                  x, y, radius);
          }
          else
          {
              return std::make_unique<GlassTrash>(
                  x, y, radius);
          }
      }
      else if (type == "obstacle")
      {
          auto o = std::make_unique<Obstacle>();

          o->x = x;
          o->y = y;
          o->radius = 0.8;

          return o;
      }

      return nullptr;
  }
};
