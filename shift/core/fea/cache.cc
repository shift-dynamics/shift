#include "shift/core/fea/cache.h"

#include <H5Cpp.h>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <random>
#include <thread>

#include "shift/core/exception.h"

/* the layout of the file and the version of the analysis: change it when
either changes (the mesher, the elements, the attachments, the reduction),
so that caches written before are computed again */
static const char* kFormat = "shift flex cache 4";

shift::core::fea::Reduction shift::core::fea::KeepModes(
  const Reduction& full, const std::vector<unsigned int>& kept) {
  const size_t s = full.static_count;
  const size_t n_full = full.elastic_count;
  for(unsigned int k : kept) {
    if(s + k >= n_full) {
      throw shift::core::Exception(
        "KeepModes: mode " + std::to_string(k) + " is kept but only " +
        std::to_string(n_full - s) + " are computed");
    }
  }
  std::vector<size_t> columns;
  for(size_t j = 0; j < s; j++) {
    columns.push_back(j);
  }
  for(unsigned int k : kept) {
    columns.push_back(s + k);
  }
  const size_t n = columns.size();
  Reduction r;
  r.positions = full.positions;
  r.masses = full.masses;
  r.elastic_count = static_cast<unsigned int>(n);
  r.static_count = full.static_count;
  r.frequencies = full.frequencies;
  r.computed_shapes = full.computed_shapes;
  r.attachment_points = full.attachment_points;
  const size_t points = full.positions.size();
  r.shapes.resize(points * n);
  for(size_t p = 0; p < points; p++) {
    for(size_t j = 0; j < n; j++) {
      r.shapes[p * n + j] = full.shapes[p * n_full + columns[j]];
    }
  }
  r.stiffness.resize(n * n);
  r.mass.resize(n * n);
  for(size_t i = 0; i < n; i++) {
    for(size_t j = 0; j < n; j++) {
      r.stiffness[i * n + j] =
        full.stiffness[columns[i] * n_full + columns[j]];
      r.mass[i * n + j] = full.mass[columns[i] * n_full + columns[j]];
    }
  }
  return r;
}

uint64_t shift::core::fea::Fnv1a(const void* data, size_t size,
                                 uint64_t hash) {
  const unsigned char* bytes = static_cast<const unsigned char*>(data);
  for(size_t i = 0; i < size; i++) {
    hash ^= bytes[i];
    hash *= 0x100000001b3ull;
  }
  return hash;
}

/* a dataset of n values of type T, in one dimension */
template <class T>
static void Write(H5::Group& group, const char* name, const T* data,
                  size_t n, const H5::PredType& type) {
  const hsize_t dims[1] = {static_cast<hsize_t>(n)};
  H5::DataSpace space(1, dims);
  H5::DataSet set = group.createDataSet(name, type, space);
  if(n > 0) {
    set.write(data, type);
  }
}

template <class T>
static std::vector<T> Read(H5::Group& group, const char* name,
                           const H5::PredType& type) {
  H5::DataSet set = group.openDataSet(name);
  hsize_t dims[1] = {0};
  set.getSpace().getSimpleExtentDims(dims);
  std::vector<T> data(dims[0]);
  if(dims[0] > 0) {
    set.read(data.data(), type);
  }
  return data;
}

static void WriteString(H5::H5Object& object, const char* name,
                        const std::string& value) {
  H5::StrType type(H5::PredType::C_S1, value.size() + 1);
  H5::Attribute attribute =
    object.createAttribute(name, type, H5::DataSpace(H5S_SCALAR));
  attribute.write(type, value.c_str());
}

static std::string ReadString(H5::H5Object& object, const char* name) {
  H5::Attribute attribute = object.openAttribute(name);
  std::string value;
  attribute.read(attribute.getStrType(), value);
  return value;
}

bool shift::core::fea::WriteFlexCache(const std::string& path,
                                      const std::string& fingerprint,
                                      const FlexCache& cache) {
  /* a name of this writer's own beside the file */
  static std::atomic<unsigned int> count{0};
  const std::string temporary =
    path + ".tmp" +
    std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) +
    "_" + std::to_string(count++);
  try {
    H5::Exception::dontPrint();
    H5::H5File file(temporary, H5F_ACC_TRUNC);
    H5::Group root = file.openGroup("/");
    WriteString(root, "format", kFormat);
    WriteString(root, "fingerprint", fingerprint);
    const Reduction& r = cache.full;
    const unsigned int counts[4] = {r.elastic_count, r.static_count,
                                    cache.node_count,
                                    static_cast<unsigned int>(cache.tet_count)};
    Write(root, "counts", counts, 4, H5::PredType::NATIVE_UINT);
    Write(root, "volume", &cache.volume, 1, H5::PredType::NATIVE_DOUBLE);
    Write(root, "positions", &r.positions.data()->x, 3 * r.positions.size(),
          H5::PredType::NATIVE_DOUBLE);
    Write(root, "masses", r.masses.data(), r.masses.size(),
          H5::PredType::NATIVE_DOUBLE);
    static_assert(sizeof(ksl_screw_t) == 6 * sizeof(double),
                  "a screw is six doubles");
    Write(root, "shapes", reinterpret_cast<const double*>(r.shapes.data()),
          6 * r.shapes.size(), H5::PredType::NATIVE_DOUBLE);
    Write(root, "stiffness", r.stiffness.data(), r.stiffness.size(),
          H5::PredType::NATIVE_DOUBLE);
    Write(root, "mass", r.mass.data(), r.mass.size(),
          H5::PredType::NATIVE_DOUBLE);
    Write(root, "frequencies", r.frequencies.data(), r.frequencies.size(),
          H5::PredType::NATIVE_DOUBLE);
    Write(root, "computed_shapes", r.computed_shapes.data(),
          r.computed_shapes.size(), H5::PredType::NATIVE_DOUBLE);
    Write(root, "attachment_points", r.attachment_points.data(),
          r.attachment_points.size(), H5::PredType::NATIVE_UINT);
    Write(root, "boundary", cache.boundary.empty() ? nullptr
                                                   : cache.boundary[0].data(),
          6 * cache.boundary.size(), H5::PredType::NATIVE_INT);
    Write(root, "boundary_face", cache.boundary_face.data(),
          cache.boundary_face.size(), H5::PredType::NATIVE_INT);
    /* the attachments' nodes, one after the other, and how many each has */
    std::vector<int> attachment_nodes, attachment_counts;
    for(const auto& nodes : cache.attachment_nodes) {
      attachment_counts.push_back(static_cast<int>(nodes.size()));
      attachment_nodes.insert(attachment_nodes.end(), nodes.begin(),
                              nodes.end());
    }
    Write(root, "attachment_nodes", attachment_nodes.data(),
          attachment_nodes.size(), H5::PredType::NATIVE_INT);
    Write(root, "attachment_node_counts", attachment_counts.data(),
          attachment_counts.size(), H5::PredType::NATIVE_INT);
    Write(root, "surface_nodes", cache.surface_nodes.data(),
          cache.surface_nodes.size(), H5::PredType::NATIVE_INT);
    Write(root, "sliding_stations", cache.sliding_stations.data(),
          cache.sliding_stations.size(), H5::PredType::NATIVE_INT);
    Write(root, "surface_stresses", cache.surface_stresses.data(),
          cache.surface_stresses.size(), H5::PredType::NATIVE_DOUBLE);
    file.close();
  } catch(H5::Exception&) {
    std::remove(temporary.c_str());
    return false;
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if(error) {
    std::remove(temporary.c_str());
    return false;
  }
  return true;
}

bool shift::core::fea::ReadFlexCache(const std::string& path,
                                     const std::string& fingerprint,
                                     FlexCache& cache) {
  std::error_code error;
  if(!std::filesystem::is_regular_file(path, error)) {
    return false;
  }
  try {
    H5::Exception::dontPrint();
    H5::H5File file(path, H5F_ACC_RDONLY);
    H5::Group root = file.openGroup("/");
    if(ReadString(root, "format") != kFormat ||
       ReadString(root, "fingerprint") != fingerprint) {
      return false;
    }
    const auto counts =
      Read<unsigned int>(root, "counts", H5::PredType::NATIVE_UINT);
    if(counts.size() != 4) {
      return false;
    }
    FlexCache c;
    Reduction& r = c.full;
    r.elastic_count = counts[0];
    r.static_count = counts[1];
    c.node_count = counts[2];
    c.tet_count = static_cast<int>(counts[3]);
    const auto volume = Read<double>(root, "volume",
                                     H5::PredType::NATIVE_DOUBLE);
    c.volume = volume.empty() ? 0.0 : volume[0];
    const auto positions =
      Read<double>(root, "positions", H5::PredType::NATIVE_DOUBLE);
    r.positions.resize(positions.size() / 3);
    for(size_t p = 0; p < r.positions.size(); p++) {
      r.positions[p] = ksl_vec3(positions[3 * p], positions[3 * p + 1],
                                positions[3 * p + 2]);
    }
    r.masses = Read<double>(root, "masses", H5::PredType::NATIVE_DOUBLE);
    const auto shapes =
      Read<double>(root, "shapes", H5::PredType::NATIVE_DOUBLE);
    r.shapes.resize(shapes.size() / 6);
    std::copy(shapes.begin(), shapes.end(),
              reinterpret_cast<double*>(r.shapes.data()));
    r.stiffness = Read<double>(root, "stiffness", H5::PredType::NATIVE_DOUBLE);
    r.mass = Read<double>(root, "mass", H5::PredType::NATIVE_DOUBLE);
    r.frequencies =
      Read<double>(root, "frequencies", H5::PredType::NATIVE_DOUBLE);
    r.computed_shapes =
      Read<double>(root, "computed_shapes", H5::PredType::NATIVE_DOUBLE);
    r.attachment_points = Read<unsigned int>(root, "attachment_points",
                                             H5::PredType::NATIVE_UINT);
    const auto boundary =
      Read<int>(root, "boundary", H5::PredType::NATIVE_INT);
    c.boundary.resize(boundary.size() / 6);
    for(size_t k = 0; k < c.boundary.size(); k++) {
      for(int i = 0; i < 6; i++) {
        c.boundary[k][i] = boundary[6 * k + i];
      }
    }
    c.boundary_face =
      Read<int>(root, "boundary_face", H5::PredType::NATIVE_INT);
    const auto attachment_nodes =
      Read<int>(root, "attachment_nodes", H5::PredType::NATIVE_INT);
    const auto attachment_counts =
      Read<int>(root, "attachment_node_counts", H5::PredType::NATIVE_INT);
    size_t next = 0;
    for(int count : attachment_counts) {
      if(count < 0 || next + size_t(count) > attachment_nodes.size()) {
        return false;
      }
      c.attachment_nodes.emplace_back(attachment_nodes.begin() + long(next),
                                      attachment_nodes.begin() +
                                        long(next + size_t(count)));
      next += size_t(count);
    }
    for(const auto& nodes : c.attachment_nodes) {
      for(int node : nodes) {
        if(node < 0 || unsigned(node) >= c.node_count) {
          return false;
        }
      }
    }
    c.surface_nodes =
      Read<int>(root, "surface_nodes", H5::PredType::NATIVE_INT);
    c.sliding_stations =
      Read<int>(root, "sliding_stations", H5::PredType::NATIVE_INT);
    c.surface_stresses =
      Read<double>(root, "surface_stresses", H5::PredType::NATIVE_DOUBLE);
    for(int node : c.surface_nodes) {
      if(node < 0 || unsigned(node) >= c.node_count) {
        return false;
      }
    }
    /* consistent sizes, or computed again */
    const size_t n = r.elastic_count;
    if(r.masses.size() != r.positions.size() ||
       r.shapes.size() != r.positions.size() * n ||
       r.stiffness.size() != n * n || r.mass.size() != n * n ||
       c.boundary_face.size() != c.boundary.size() ||
       c.surface_stresses.size() != 6 * c.surface_nodes.size() * n ||
       r.computed_shapes.size() !=
         3 * size_t(c.node_count) * r.frequencies.size() ||
       c.node_count > r.positions.size()) {
      return false;
    }
    cache = std::move(c);
  } catch(H5::Exception&) {
    return false;
  }
  return true;
}
