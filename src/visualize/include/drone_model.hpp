#pragma once

#include <pangolin/geometry/geometry.h>
#include <pangolin/pangolin.h>
#include "drone_pose.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

// The original Ogre asset is converted offline to PLY. Only Pangolin is needed at runtime.
class DroneModel
{
 public:
  explicit DroneModel(const std::string &filename)
  {
    const auto                   geometry = pangolin::LoadGeometry(filename);
    std::vector<Eigen::Vector3f> vertices;
    for (const auto &buffer : geometry.buffers)
    {
      const auto vertex = buffer.second.attributes.find("vertex");
      if (vertex != buffer.second.attributes.end())
      {
        const auto &data = std::get<pangolin::Image<float>>(vertex->second);
        if (data.w != 3)
        {
          throw std::runtime_error("Drone vertices must have three components");
        }
        for (size_t i = 0; i < data.h; ++i)
        {
          vertices.emplace_back(Eigen::Map<const Eigen::Vector3f>(data.RowPtr(i)));
        }
        break;
      }
    }
    std::vector<std::array<size_t, 3>> triangles;
    for (const auto &object : geometry.objects)
    {
      auto indices = object.second.attributes.find("tris");
      for (const auto *name : {"vertex_index", "vertex_indices"})
      {
        if (indices == object.second.attributes.end())
        {
          indices = object.second.attributes.find(name);
        }
      }
      if (indices != object.second.attributes.end())
      {
        std::visit(
            [&](const auto &data)
            {
              if (data.w != 3)
              {
                throw std::runtime_error("Drone faces must have three indices");
              }
              for (size_t i = 0; i < data.h; ++i)
              {
                const auto *row = data.RowPtr(i);
                triangles.push_back({static_cast<size_t>(row[0]), static_cast<size_t>(row[1]),
                                     static_cast<size_t>(row[2])});
              }
            },
            indices->second);
      }
    }
    if (vertices.empty() || triangles.empty())
    {
      throw std::runtime_error("Drone PLY must contain vertices and triangular faces: " + filename);
    }
    for (const auto &triangle : triangles)
    {
      for (size_t index : triangle)
      {
        if (index >= vertices.size() || !vertices[index].allFinite())
        {
          throw std::runtime_error("Invalid drone mesh vertex/index");
        }
      }
    }
    list_ = glGenLists(1);
    if (!list_)
    {
      throw std::runtime_error("Cannot create drone display list");
    }
    glNewList(list_, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for (const auto &triangle : triangles)
    {
      const auto &a = vertices[triangle[0]], &b = vertices[triangle[1]], &c = vertices[triangle[2]];
      const Eigen::Vector3f normal = (b - a).cross(c - a).stableNormalized();
      glNormal3fv(normal.data());
      glVertex3fv(a.data());
      glVertex3fv(b.data());
      glVertex3fv(c.data());
    }
    glEnd();
    glEndList();
  }

  ~DroneModel() { glDeleteLists(list_, 1); }

  DroneModel(const DroneModel &)            = delete;
  DroneModel &operator=(const DroneModel &) = delete;

  void draw(const DronePose &pose, double scale, const pangolin::Colour &body,
            const pangolin::Colour &front, bool show_axes, double axis_length) const
  {
    if (!pose.valid)
    {
      return;
    }
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT | GL_LINE_BIT);
    glPushMatrix();
    glMultMatrixd(pose.transform.data());
    glScaled(scale, scale, scale);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    const GLfloat ambient[] = {.45f, .45f, .45f, 1.f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glColor3f(body.r, body.g, body.b);
    glCallList(list_);
    glDisable(GL_LIGHTING);
    // Arrow along body +X: this remains visible on the nearly symmetric quadrotor.
    glLineWidth(3);
    glColor3f(front.r, front.g, front.b);
    glBegin(GL_LINES);
    glVertex3d(0, 0, .07);
    glVertex3d(.28, 0, .07);
    glVertex3d(.28, 0, .07);
    glVertex3d(.21, .045, .07);
    glVertex3d(.28, 0, .07);
    glVertex3d(.21, -.045, .07);
    glEnd();
    if (show_axes)
    {
      glLineWidth(2);
      glBegin(GL_LINES);
      for (int i = 0; i < 3; ++i)
      {
        glColor3f(i == 0, i == 1, i == 2);
        glVertex3d(0, 0, 0);
        std::array<double, 3> tip = {0, 0, 0};
        tip[i]                    = axis_length;
        glVertex3dv(tip.data());
      }
      glEnd();
    }
    glPopMatrix();
    glPopAttrib();
  }

 private:
  GLuint list_ = 0;
};
