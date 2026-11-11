// Offline conversion only: the Pangolin viewer does not depend on Ogre or Assimp.
#include <Ogre.h>
#include <OgreDefaultHardwareBufferManager.h>
#include <assimp/postprocess.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>

#include <fstream>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    std::cerr << "Usage: convert_drone_mesh input.mesh intermediate.mesh output.ply\n";
    return 1;
  }
  try
  {
    Ogre::LogManager log;
    log.createLog("", true, false, true);
    Ogre::ResourceGroupManager         resources;
    Ogre::DefaultHardwareBufferManager buffers;
    Ogre::MeshManager                  meshes;
    Ogre::LodStrategyManager           lod;
    auto                               mesh =
        meshes.createManual("drone", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    std::ifstream file(argv[1], std::ios::binary);
    if (!file)
    {
      throw std::runtime_error("Cannot open input mesh");
    }
    Ogre::DataStreamPtr  stream(new Ogre::FileStreamDataStream(&file, false));
    Ogre::MeshSerializer serializer;
    serializer.importMesh(stream, mesh.get());
    serializer.exportMesh(mesh.get(), argv[2], Ogre::MESH_VERSION_1_8);

    Assimp::Importer importer;
    const auto      *scene =
        importer.ReadFile(argv[2], aiProcess_Triangulate | aiProcess_PreTransformVertices |
                                       aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);
    if (!scene)
    {
      throw std::runtime_error(importer.GetErrorString());
    }
    Assimp::Exporter exporter;
    if (exporter.Export(scene, "plyb", argv[3]) != AI_SUCCESS)
    {
      throw std::runtime_error(exporter.GetErrorString());
    }
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
