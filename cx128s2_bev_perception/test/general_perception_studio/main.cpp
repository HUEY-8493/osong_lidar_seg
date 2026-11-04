#define GLFW_INCLUDE_NONE
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "implot.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <png.h>
#include <set>
#include <signal.h>
#include <sys/file.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>
using J = nlohmann::json;
namespace fs = std::filesystem;
static const J emptyObject = J::object(), emptyArray = J::array();
static const J &obj(const J &j, const char *k) {
  auto i = j.find(k);
  return i != j.end() && i->is_object() ? *i : emptyObject;
}
static const J &arr(const J &j, const char *k) {
  auto i = j.find(k);
  return i != j.end() && i->is_array() ? *i : emptyArray;
}
static std::string str(const J &j, const char *k,
                       const std::string &fallback = "") {
  auto i = j.find(k);
  if (i == j.end() || i->is_null())
    return fallback;
  return i->is_string() ? i->get<std::string>() : i->dump();
}
static std::string brief(const J &j, const char *k, const std::string &fallback="-") {
  auto i=j.find(k); if(i==j.end()||i->is_null())return fallback;
  if(i->is_number_float()){char b[64];std::snprintf(b,sizeof(b),"%.3f",i->get<double>());return b;}
  return str(j,k,fallback);
}
static double num(const J &j, const char *k, double fallback = 0) {
  auto i = j.find(k);
  if (i == j.end() || i->is_null())
    return fallback;
  try {
    return i->is_number() ? i->get<double>() : std::stod(i->get<std::string>());
  } catch (...) {
    return fallback;
  }
}
static bool yes(const J &j, const char *k) {
  auto i = j.find(k);
  return i != j.end() && (*i == true || *i == 1 || *i == "1");
}
static std::string readText(const fs::path &p) {
  std::ifstream f(p);
  return std::string(std::istreambuf_iterator<char>(f), {});
}
static void writeJson(const fs::path &p, const J &j) {
  fs::create_directories(p.parent_path());
  std::ofstream(p) << j.dump(2);
}
static double now() { return glfwGetTime(); }
static ImU32 col(float r, float g, float b, float a = 1) {
  return ImGui::ColorConvertFloat4ToU32({r, g, b, a});
}
static void text(const std::string &s) { ImGui::TextUnformatted(s.c_str()); }
static void wrapped(const std::string &s) {
  ImGui::PushTextWrapPos();
  text(s);
  ImGui::PopTextWrapPos();
}
static void raw(const char *title, const J &j) {
  if (ImGui::TreeNode(title)) {
    wrapped(j.dump(2));
    ImGui::TreePop();
  }
}

static void selectPanel(const char *name) {
  ImGui::SetWindowFocus(name);
  if (auto *window = ImGui::FindWindowByName(name))
    if (auto *node = window->DockNode)
      if (node->TabBar) {
        node->TabBar->SelectedTabId = node->TabBar->NextSelectedTabId =
            window->TabId;
        node->SelectedTabId = window->TabId;
      }
}
class Worker {
  pid_t pid = -1;
  FILE *output = nullptr;
  int input = -1;
  std::thread reader;
  std::mutex mutex;
  std::deque<J> responses;
  int serial = 0;

public:
  Worker(const fs::path &state, const std::vector<std::string> &roots) {
    int in[2], out[2];
    if (pipe(in) || pipe(out))
      throw std::runtime_error("Worker pipe failed");
    pid = fork();
    if (pid < 0)
      throw std::runtime_error("Worker fork failed");
    if (pid == 0) {
      dup2(in[0], STDIN_FILENO);
      dup2(out[1], STDOUT_FILENO);
      for (int f : {in[0], in[1], out[0], out[1]})
        close(f);
      int log = open((state / "studio_worker.log").c_str(),
                     O_WRONLY | O_CREAT | O_APPEND, 0644);
      if (log >= 0) {
        dup2(log, STDERR_FILENO);
        close(log);
      }
      std::vector<std::string> args = {
          "python3", STUDIO_SOURCE_DIR "/worker.py", "--state", state.string()};
      for (auto &root : roots) {
        args.push_back("--dataset-root");
        args.push_back(root);
      }
      std::vector<char *> pointers;
      for (auto &arg : args)
        pointers.push_back(arg.data());
      pointers.push_back(nullptr);
      execvp("python3", pointers.data());
      _exit(127);
    }
    close(in[0]);
    close(out[1]);
    input = in[1];
    fcntl(input, F_SETFD, FD_CLOEXEC);
    output = fdopen(out[0], "r");
    reader = std::thread([this] {
      char *line = nullptr;
      size_t n = 0;
      while (getline(&line, &n, output) > 0) {
        try {
          J j = J::parse(line);
          std::lock_guard<std::mutex> l(mutex);
          responses.push_back(std::move(j));
        } catch (...) {
        }
      }
      free(line);
      std::lock_guard<std::mutex> l(mutex);
      responses.push_back(
          {{"id", -1},
           {"ok", false},
           {"error", "Backend worker exited; see studio_worker.log"}});
    });
  }
  int send(J j) {
    j["id"] = ++serial;
    std::string s = j.dump() + "\n";
    size_t p = 0;
    while (p < s.size()) {
      ssize_t n = write(input, s.data() + p, s.size() - p);
      if (n <= 0)
        throw std::runtime_error("Worker connection closed");
      p += n;
    }
    return serial;
  }
  bool poll(J &j) {
    std::lock_guard<std::mutex> l(mutex);
    if (responses.empty())
      return false;
    j = std::move(responses.front());
    responses.pop_front();
    return true;
  }
  ~Worker() {
    if (input >= 0) {
      close(input);
      input = -1;
    }
    if (reader.joinable())
      reader.join();
    if (output)
      fclose(output);
    if (pid > 0)
      waitpid(pid, nullptr, 0);
  }
};

struct Camera {
  double x = 0, y = 0, width = 100;
};
struct Vertex {
  float x, y, r, g, b;
};
class CloudRenderer {
  GLuint program = 0, vbo = 0, fbo = 0, texture = 0, vao = 0;
  int w = 0, h = 0;
  size_t n = 0;
  static GLuint shader(GLenum t, const char *s) {
    GLuint v = glCreateShader(t);
    glShaderSource(v, 1, &s, nullptr);
    glCompileShader(v);
    GLint ok;
    glGetShaderiv(v, GL_COMPILE_STATUS, &ok);
    if (!ok)
      throw std::runtime_error("Point shader failed");
    return v;
  }

public:
  CloudRenderer() {
    const char *vs =
        "#version 130\nin vec2 position;in vec3 color;out vec3 tint;uniform "
        "vec3 camera;uniform float aspect;uniform float point_size;void "
        "main(){gl_Position=vec4((position.x-camera.x)*2.0/"
        "camera.z,(position.y-camera.y)*2.0/(camera.z/"
        "aspect),0,1);gl_PointSize=point_size;tint=color;}";
    const char *fs = "#version 130\nin vec3 tint;out vec4 output_color;void "
                     "main(){output_color=vec4(tint,1);}";
    auto v = shader(GL_VERTEX_SHADER, vs), f = shader(GL_FRAGMENT_SHADER, fs);
    program = glCreateProgram();
    glAttachShader(program, v);
    glAttachShader(program, f);
    glBindAttribLocation(program, 0, "position");
    glBindAttribLocation(program, 1, "color");
    glLinkProgram(program);
    GLint ok;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok)
      throw std::runtime_error("Point shader link failed");
    glDeleteShader(v);
    glDeleteShader(f);
    glGenBuffers(1, &vbo);
    glGenVertexArrays(1, &vao);
    glGenFramebuffers(1, &fbo);
    glGenTextures(1, &texture);
  }
  ~CloudRenderer() {
    glDeleteTextures(1, &texture);
    glDeleteFramebuffers(1, &fbo);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
  }
  void upload(const std::vector<Vertex> &v) {
    n = v.size();
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vertex)),
                 v.data(), GL_STATIC_DRAW);
  }
  GLuint render(int width, int height, const Camera &c, float pointSize) {
    width = std::max(2, width);
    height = std::max(2, height);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glBindTexture(GL_TEXTURE_2D, texture);
    if (w != width || h != height) {
      w = width;
      h = height;
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE,
                   nullptr);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                             GL_TEXTURE_2D, texture, 0);
    }
    glViewport(0, 0, w, h);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glClearColor(.075f, .092f, .083f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(program);
    glUniform3f(glGetUniformLocation(program, "camera"), c.x, c.y, c.width);
    glUniform1f(glGetUniformLocation(program, "aspect"), float(w) / h);
    glUniform1f(glGetUniformLocation(program, "point_size"), pointSize);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)(2 * sizeof(float)));
    glDrawArrays(GL_POINTS, 0, GLsizei(n));
    glBindVertexArray(0);
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return texture;
  }
};

struct Pending {
  std::string op;
  int epoch = 0, frame = 0, side = 0, track = -1;
  std::string run0, run1;
};
class Studio {
public:
  fs::path state;
  GLFWwindow *window;
  Worker worker;
  CloudRenderer clouds[2];
  J catalog = J::array(), opened = J::object(), bundle = J::object(),
    jobs = J::array(), events = J::array(),
    graphs[2] = {J::object(), J::object()}, whatif = J::object();
  J buffer = J::object();
  double lastBuffer = 0;
  bool bufferInitialized = false, autoPrefetch = true;
  bool showBuffer = false;
  int bufferMB = 512, lookahead = 2;
  std::map<int, Pending> requests;
  std::map<std::string, std::string> edits;
  std::string root = "local", dataset = "20261023", mode = "Explore", error,
              status = "Opening workspace", runs[2], runLabels[2],
              cameraNotice = "Fit visible to begin";
  int frame = 80, total = 1, track = 6, cluster = -1, anchor = 0, epoch = 0,
      requestedFrame = -1, rangeStart = 80, rangeEnd = 80, fitTarget = 0;
  bool focusPreset = false, fitGraphPending = false, graphRange = false,
       trajectoryBoundary = true, trajectoryFit = true, validateTrackingLines = false;
  int validationTargetFrame = -1;
  ImVec2 validationTimelinePoint{}, validationTrajectoryPoint{};
  double graphMin = 1, graphMax = 2;
  int graphClickFrame = -1;
  bool playing = false, follow = false, focusOnly = false, dark = false,
       layout = true, needsUpload = true, showRaw = false, fullFocus = false,
       quit = false;
  bool pointLayer = true, foreground = true, roiPoints = true, outside = true,
       clusterLayer = true, trackLayer = true, geometryLayer = true,
       memoryLayer = true, kfLayer = true, roiLayer = true, sensorLayer = true,
       grid = true, fovLayer = true;
  bool actualFilter = true, predictionFilter = true, dormantFilter = true,
       publicFilter = true, nonpublicFilter = true, rejectedFilter = true,
       matchedFilter = true, vehicleLikeFilter = true, unknownFilter = true,
       rejectedClusterFilter = true, unclassifiedFilter = true;
  float pointSize = 1.5f, speed = 1;
  Camera camera;
  int visiblePoints[2] = {0, 0};
  double lastPlay = 0, lastJobs = 0;
  char label[128] = "candidate", eventSearch[128] = "", paramSearch[128] = "";
  int metric = 0;
  std::string metricKey = "angular_bins";
  std::string exportRequest;
  J validation = J::object();
  Studio(GLFWwindow *win, const fs::path &p,
         const std::vector<std::string> &roots)
      : state(p), window(win), worker(p, roots) {
    fs::path prefs = state / "studio_settings.json";
    if (fs::exists(prefs)) {
      try {
        J j = J::parse(readText(prefs));
        dark = j.value("dark", false);
        mode = j.value("mode", mode);
        root = j.value("root", root);
        dataset = j.value("dataset", dataset);
        frame = j.value("frame", frame);
        track = j.value("track", track);
        camera = {j.value("camera_x", 0.), j.value("camera_y", 0.),
                  j.value("camera_width", 100.)};
        if (j.contains("layers"))
          setLayers(j["layers"]);
      } catch (...) {
      }
    }
    theme();
    ask("catalog");
    ask("jobs");
  }
  ~Studio() { save(); }
  J layers() const {
    return {{"points", pointLayer},
            {"foreground", foreground},
            {"roi_nonforeground", roiPoints},
            {"outside_roi", outside},
            {"clusters", clusterLayer},
            {"tracks", trackLayer},
            {"geometry", geometryLayer},
            {"memory", memoryLayer},
            {"kf", kfLayer},
            {"roi", roiLayer},
            {"sensor", sensorLayer},
            {"grid", grid},
            {"fov", fovLayer},
            {"actual", actualFilter},
            {"prediction", predictionFilter},
            {"dormant", dormantFilter},
            {"public", publicFilter},
            {"nonpublic", nonpublicFilter},
            {"rejected_tracks", rejectedFilter},
            {"matched", matchedFilter},
            {"vehicle_like", vehicleLikeFilter},
            {"unknown_like", unknownFilter},
            {"rejected_clusters", rejectedClusterFilter},
            {"unclassified", unclassifiedFilter}};
  }
  void setLayers(const J &j) {
    for (auto p : std::vector<std::pair<const char *, bool *>>{
             {"points", &pointLayer},
             {"foreground", &foreground},
             {"roi_nonforeground", &roiPoints},
             {"outside_roi", &outside},
             {"clusters", &clusterLayer},
             {"tracks", &trackLayer},
             {"geometry", &geometryLayer},
             {"memory", &memoryLayer},
             {"kf", &kfLayer},
             {"roi", &roiLayer},
             {"sensor", &sensorLayer},
             {"grid", &grid},
             {"fov", &fovLayer},
             {"actual", &actualFilter},
             {"prediction", &predictionFilter},
             {"dormant", &dormantFilter},
             {"public", &publicFilter},
             {"nonpublic", &nonpublicFilter},
             {"rejected_tracks", &rejectedFilter},
             {"matched", &matchedFilter},
             {"vehicle_like", &vehicleLikeFilter},
             {"unknown_like", &unknownFilter},
             {"rejected_clusters", &rejectedClusterFilter},
             {"unclassified", &unclassifiedFilter}})
      if (j.contains(p.first))
        *p.second = j[p.first].get<bool>();
    needsUpload = true;
  }
  void save() {
    writeJson(state / "studio_settings.json", {{"dark", dark},
                                               {"mode", mode},
                                               {"root", root},
                                               {"dataset", dataset},
                                               {"frame", frame},
                                               {"track", track},
                                               {"camera_x", camera.x},
                                               {"camera_y", camera.y},
                                               {"camera_width", camera.width},
                                               {"layers", layers()}});
  }
  void theme() {
    if (dark)
      ImGui::StyleColorsDark();
    else
      ImGui::StyleColorsLight();
    auto &s = ImGui::GetStyle();
    s.WindowRounding = 6;
    s.FrameRounding = 5;
    s.PopupRounding = 6;
    s.TabRounding = 5;
    s.WindowBorderSize = 0;
    s.FrameBorderSize = 0;
    s.WindowPadding = {16, 14};
    s.FramePadding = {9, 6};
    s.ItemSpacing = {10, 8};
    s.CellPadding = {8, 5};
    auto *c = s.Colors;
    c[ImGuiCol_Text] =
        dark ? ImVec4(.88, .89, .86, 1) : ImVec4(.16, .18, .16, 1);
    c[ImGuiCol_WindowBg] =
        dark ? ImVec4(.12, .135, .125, 1) : ImVec4(.97, .965, .945, 1);
    c[ImGuiCol_DockingEmptyBg] =
        dark ? ImVec4(.09, .10, .09, 1) : ImVec4(.91, .91, .88, 1);
    c[ImGuiCol_FrameBg] =
        dark ? ImVec4(.19, .21, .19, 1) : ImVec4(.90, .91, .88, 1);
    c[ImGuiCol_Button] =
        dark ? ImVec4(.22, .28, .24, 1) : ImVec4(.84, .89, .84, 1);
    c[ImGuiCol_ButtonHovered] = {.36, .57, .43, 1};
    c[ImGuiCol_ButtonActive] = {.22, .46, .32, 1};
    c[ImGuiCol_CheckMark] = {.24, .57, .37, 1};
    c[ImGuiCol_SliderGrab] = {.3, .56, .4, 1};
    c[ImGuiCol_Header] =
        dark ? ImVec4(.24, .32, .27, 1) : ImVec4(.82, .88, .82, 1);
    c[ImGuiCol_HeaderHovered] = {.4, .6, .46, .6};
    c[ImGuiCol_Tab] = c[ImGuiCol_FrameBg];
    c[ImGuiCol_TabSelected] = c[ImGuiCol_Button];
    c[ImGuiCol_TabHovered] = c[ImGuiCol_ButtonHovered];
    c[ImGuiCol_TitleBgActive] = c[ImGuiCol_WindowBg];
    c[ImGuiCol_Separator] =
        dark ? ImVec4(.25, .28, .25, 1) : ImVec4(.83, .84, .8, 1);
    c[ImGuiCol_TabSelectedOverline] = {.3f, .57f, .39f, 1};
    c[ImGuiCol_TabDimmedSelected] = c[ImGuiCol_Button];
    c[ImGuiCol_TabDimmedSelectedOverline] = c[ImGuiCol_TabSelectedOverline];
    c[ImGuiCol_DockingPreview] = {.3f, .6f, .4f, .6f};
    c[ImGuiCol_NavCursor] = {.3f, .6f, .4f, 1};
    c[ImGuiCol_TextSelectedBg] = {.3f, .6f, .4f, .3f};
    c[ImGuiCol_ResizeGripHovered] = {.3f, .6f, .4f, .6f};
    c[ImGuiCol_ResizeGripActive] = {.3f, .6f, .4f, 1};
    c[ImGuiCol_TableHeaderBg] =
        dark ? ImVec4(.21f, .24f, .21f, 1) : ImVec4(.86f, .89f, .85f, 1);
    c[ImGuiCol_TableBorderStrong] = c[ImGuiCol_Separator];
    c[ImGuiCol_TableBorderLight] = c[ImGuiCol_Separator];
    c[ImGuiCol_TabDimmed] = c[ImGuiCol_FrameBg];
    static const ImVec4 plotColors[] = {{.28f, .65f, .4f, 1},
                                        {.78f, .57f, .35f, 1},
                                        {.63f, .48f, .72f, 1},
                                        {.68f, .7f, .6f, 1}};
    static int palette = ImPlot::AddColormap("Studio neutral", plotColors, 4);
    ImPlot::GetStyle().Colormap = palette;
    ImPlot::GetStyle().PlotPadding = {12, 10};
    ImPlot::GetStyle().FitPadding = {.05f, .15f};
  }
  int ask(const std::string &op, J extras = J::object(), int side = 0) {
    extras["op"] = op;
    if (op != "catalog" && op != "jobs" && op != "restore" && op != "start" &&
        op != "cancel") {
      extras["root"] = root;
      extras["dataset"] = dataset;
    }
    int id = worker.send(extras);
    requests[id] = {op, epoch, frame, side, track, runs[0], runs[1]};
    return id;
  }
  const J &data() const { return obj(bundle, "data"); }
  const J &st(int side = 0) const {
    const J &s = arr(bundle, "states");
    return s.size() > size_t(side) ? s[side] : emptyObject;
  }
  const J *selected(int side = -1) const {
    if (side < 0)
      side = anchor;
    for (auto &t : arr(st(side), "display_tracks"))
      if (int(num(t, "id", -1)) == track)
        return &t;
    return nullptr;
  }
  bool loaded() const { return num(data(), "frame", -1) == frame; }
  void open() {
    epoch++;
    whatif = J::object();
    requestedFrame = -1;
    bundle = J::object();
    opened = J::object();
    error.clear();
    status = "Loading dataset";
    graphs[0] = graphs[1] = J::object();
    events = J::array();
    runs[0] = runs[1] = "";
    cluster = -1;
    ask("open");
  }
  void fetch() {
    if (opened.empty() || requestedFrame == frame)
      return;
    for (auto &p : requests)
      if (p.second.op == "frame")
        return;
    requestedFrame = frame;
    ask("frame", {{"frame", frame}, {"runs", J::array({runs[0], runs[1]})}});
    status = "Loading stored points";
  }
  void loadAnalysis() {
    if (!runs[anchor].empty())
      ask("events", {{"run", runs[anchor]}});
    if (track >= 0)
      for (int i = 0; i < 2; i++)
        if (!runs[i].empty())
          ask("graph", {{"run", runs[i]}, {"track", track}}, i);
  }
  void pickTrack(int id) {
    track = id;
    cluster = -1;
    needsUpload = true;
    graphs[0] = graphs[1] = J::object();
    loadAnalysis();
  }
  void seek(int value) {
    frame = std::clamp(value, 1, total);
    cluster = -1;
    whatif = J::object();
    fetch();
  }
  void process() {
    J response;
    while (worker.poll(response)) {
      int id = response.value("id", -1);
      auto it = requests.find(id);
      if (it == requests.end()) {
        if (id == -1)
          error = str(response, "error");
        continue;
      }
      Pending p = it->second;
      requests.erase(it);
      if (p.epoch != epoch && p.op != "jobs" && p.op != "start" &&
          p.op != "restore" && p.op != "catalog")
        continue;
      if (!yes(response, "ok")) {
        error = str(response, "error");
        status = "Attention";
        continue;
      }
      const J &r = response["result"];
      if (p.op.rfind("buffer-", 0) == 0) {
        buffer = r;
        if (!bufferInitialized || p.op == "buffer-config") {
          bufferInitialized = true;
          autoPrefetch = yes(r, "auto");
          bufferMB = int(num(r, "limit", 512 * 1024 * 1024)) / 1024 / 1024;
          lookahead = int(num(r, "lookahead", 2));
        }
      } else if (p.op == "catalog") {
        catalog = r;
        bool found = false;
        for (auto &d : catalog)
          if (str(d, "root") == root && str(d, "key") == dataset &&
              str(d, "status") == "Prepared")
            found = true;
        if (!found)
          for (auto &d : catalog)
            if (str(d, "status") == "Prepared") {
              root = str(d, "root");
              dataset = str(d, "key");
              break;
            }
        open();
      } else if (p.op == "open") {
        opened = r;
        total = arr(opened, "rows").size();
        graphMin = 1; graphMax = std::max(2, total);
        frame = std::clamp(frame, 1, std::max(1, total));
        rangeStart = rangeEnd = frame;
        const J &rr = arr(opened, "runs");
        for (int i = 0; i < 2; i++) {
          if (!rr.empty()) {
            size_t n = std::min(size_t(i), rr.size() - 1);
            runs[i] = str(rr[n], "id");
            runLabels[i] = str(rr[n], "label");
            for (auto &q : rr)
              if (str(q, "label") ==
                  (i ? "kf_noise_candidate" : "debug_baseline")) {
                runs[i] = str(q, "id");
                runLabels[i] = str(q, "label");
              }
          }
        }
        fetch();
        loadAnalysis();
      } else if (p.op == "frame") {
        if (p.frame == frame && p.run0 == runs[0] && p.run1 == runs[1]) {
          bundle = r;
          if (r.contains("buffer"))
            buffer = r["buffer"];
          needsUpload = true;
          status = "Ready";
          error.clear();
          if (fitGraphPending) {
            fitSelected(); fitGraphPending = false;
          } else if (follow)
            focusTrack(false);
        } else {
          requestedFrame = -1;
          fetch();
        }
      } else if (p.op == "events" && (anchor ? p.run1 : p.run0) == runs[anchor])
        events = r.value("events", J::array());
      else if (p.op == "graph" && p.track == track &&
               (p.side ? p.run1 : p.run0) == runs[p.side])
        { graphs[p.side] = r; trajectoryFit = true; }
      else if (p.op == "jobs") {
        jobs = r;
        lastJobs = now();
      } else if (p.op == "start") {
        status = "Experiment queued";
        ask("jobs");
      } else if (p.op == "cancel")
        ask("jobs");
      else if (p.op == "restore") {
        if (r.contains("whatif")) {
          bool different =
              root != str(r, "root") || dataset != str(r, "dataset");
          root = str(r, "root");
          dataset = str(r, "dataset");
          frame = int(num(r["whatif"], "frame", frame));
          if (different)
            open();
          else {
            requestedFrame = -1;
            fetch();
          }
          whatif = r["whatif"];
        } else {
          root = str(r, "root");
          dataset = str(r, "dataset");
          restoreRun = str(r, "id");
          frame = int(num(obj(r, "detail_range"), "0", frame));
          if (r.contains("detail_range") && r["detail_range"].is_array())
            frame = r["detail_range"][0].get<int>();
          open();
        }
        for (auto i = obj(r, "overrides").begin();
             i != obj(r, "overrides").end(); ++i)
          edits[i.key()] = i.value().dump();
      }
    }
    if (!restoreRun.empty() && !opened.empty()) {
      for (auto &r : arr(opened, "runs"))
        if (str(r, "id") == restoreRun) {
          runs[1] = restoreRun;
          runLabels[1] = str(r, "label");
          restoreRun.clear();
          requestedFrame = -1;
          fetch();
          loadAnalysis();
          break;
        }
    }
    if (playing && loaded() && now() - lastPlay > .1 / speed) {
      lastPlay = now();
      if (frame < total)
        seek(frame + 1);
      else
        playing = false;
    }
    if (now() - lastJobs > 2) {
      bool pending = false;
      for (auto &r : requests)
        if (r.second.op == "jobs")
          pending = true;
      if (!pending)
        ask("jobs");
    }
    if (now() - lastBuffer > .5) {
      bool pending = false;
      for (auto &q : requests)
        if (q.second.op == "buffer-status")
          pending = true;
      if (!pending) {
        ask("buffer-status");
        lastBuffer = now();
      }
    }
    fetch();
  }
  std::string restoreRun;
  void dock() {
    ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x, vp->WorkPos.y + 90});
    ImGui::SetNextWindowSize({vp->WorkSize.x, vp->WorkSize.y - 90});
    ImGui::SetNextWindowViewport(vp->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin(
        "Workspace", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleVar();
    ImGuiID id = ImGui::GetID("StudioDock");
    if (layout) {
      layout = false;
      focusPreset = true;
      ImGui::DockBuilderRemoveNode(id);
      ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
      ImGui::DockBuilderSetNodeSize(id, {vp->WorkSize.x, vp->WorkSize.y - 90});
      ImGuiID center = id, right, bottom;
      ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, mode == "Compare" ? .50f : .38f, &bottom,
                                  &center);
      ImGui::DockBuilderSplitNode(center, ImGuiDir_Right,
                                  mode == "Tune" ? .36f : .28f, &right,
                                  &center);
      ImGui::DockBuilderDockWindow("BEV / Before", center);
      if (mode == "Compare") {
        ImGuiID after;
        ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, .5f, &after,
                                    &center);
        ImGui::DockBuilderDockWindow("BEV / After", after);
      } else
        ImGui::DockBuilderDockWindow("BEV / After", center);
      ImGuiID lowerRight;
      ImGui::DockBuilderSplitNode(right, ImGuiDir_Down,
                                  mode == "Tune" ? .35f : .48f, &lowerRight,
                                  &right);
      ImGui::DockBuilderDockWindow("Track inspector",
                                   mode == "Tune" ? lowerRight : right);
      ImGui::DockBuilderDockWindow("Geometry", right);
      ImGui::DockBuilderDockWindow("FOV / Depth", right);
      ImGui::DockBuilderDockWindow("Decision / Gates", lowerRight);
      ImGui::DockBuilderDockWindow("Parameter Lab",
                                   mode == "Tune" ? right : lowerRight);
      ImGui::DockBuilderDockWindow("Experiments", lowerRight);
      for (auto name : {"Timeline", "ImPlot metrics", "World XY history",
                        "Events / Failure explorer", "A/B comparison"})
        ImGui::DockBuilderDockWindow(name, bottom);
      ImGui::DockBuilderFinish(id);
    }
    ImGui::DockSpace(id, {0, 0});
    ImGui::End();
  }
  void bufferPanel() {
    if (ImGui::SmallButton("Buffer") || showBuffer) {
      showBuffer = false;
      ImGui::OpenPopup("Frame buffer");
    }
    ImGui::SetNextWindowSize({470, 0}, ImGuiCond_Appearing);
    if (ImGui::BeginPopup("Frame buffer")) {
      ImGui::Text("RAM: %.1f / %.0f MiB | %d frames",
                  num(buffer, "bytes") / 1048576.,
                  num(buffer, "limit", 536870912) / 1048576.,
                  int(num(buffer, "frames")));
      ImGui::Text("Run / Trace RAM: %.1f / 128 MiB",
                  num(buffer, "trace_bytes") / 1048576.);
      ImGui::Text("RAM hits %d / loads %d", int(num(buffer, "hits")),
                  int(num(buffer, "misses")));
      text("Preparation: " + str(buffer, "status", "idle"));
      if (num(buffer, "total") > 0) {
        float progress = float(num(buffer, "done") / num(buffer, "total"));
        ImGui::ProgressBar(progress, {340, 8}, "");
        ImGui::Text("%d / %d frames prepared", int(num(buffer, "done")),
                    int(num(buffer, "total")));
      }
      if (!str(buffer, "error").empty())
        wrapped(str(buffer, "error"));
      bool changed =
          ImGui::Checkbox("Auto prefetch next NAS frames", &autoPrefetch);
      ImGui::SetNextItemWidth(110);
      changed |= ImGui::SliderInt("Lookahead", &lookahead, 1, 16);
      ImGui::SetNextItemWidth(110);
      ImGui::InputInt("Frame RAM limit (MiB)", &bufferMB, 128, 256);
      if (ImGui::Button("Apply memory limit") || changed)
        ask("buffer-config", {{"auto", autoPrefetch},
                              {"lookahead", lookahead},
                              {"limit_mb", std::clamp(bufferMB, 32, 2048)}});
      ImGui::Separator();
      ImGui::Text("Prepare detail range: %d - %d", rangeStart, rangeEnd);
      if (ImGui::Button("Preload selected range"))
        ask("buffer-start", {{"start", rangeStart}, {"end", rangeEnd}});
      ImGui::SameLine();
      if (ImGui::Button("Stop preload"))
        ask("buffer-cancel");
      if (ImGui::Button("Clear RAM"))
        ask("buffer-config", {{"clear", true}, {"auto", false}}),
            autoPrefetch = false;
      wrapped("Prepares display frames in RAM and the existing local LOD "
              "cache. Larger ranges use LRU eviction. Source data and Replay "
              "are unchanged. Stop finishes at most the current frame.");
      ImGui::EndPopup();
    }
  }
  void header() {
    auto *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize({vp->WorkSize.x, 90});
    ImGui::Begin("Studio header", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextColored({.3, .62, .42, 1}, "GENERAL");
    ImGui::SameLine();
    text("Perception Studio");
    ImGui::SameLine(240);
    ImGui::SetNextItemWidth(std::min(360.f, vp->WorkSize.x * .32f));
    std::string title = dataset + " / " + root;
    if (ImGui::BeginCombo("##dataset", title.c_str())) {
      for (auto &d : catalog) {
        bool ready = str(d, "status") == "Prepared";
        ImGui::BeginDisabled(!ready);
        std::string title = str(d, "key") + " / " + str(d, "root") +
                            (!ready ? " (unavailable)" : "");
        if (ImGui::Selectable(title.c_str(), str(d, "key") == dataset &&
                                                 str(d, "root") == root)) {
          dataset = str(d, "key");
          root = str(d, "root");
          open();
        }
        ImGui::EndDisabled();
        if (!ready && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          ImGui::SetTooltip("%s", str(d, "error").c_str());
      }
      ImGui::EndCombo();
    }
    for (auto m : {"Explore", "Compare", "Tune"}) {
      ImGui::SameLine();
      if (ImGui::Selectable(m, mode == m, 0, {76, 24})) {
        mode = m;
        layout = true;
      }
    }
    if (ImGui::SmallButton("Workspace"))
      ImGui::OpenPopup("Workspace settings");
    if (ImGui::BeginPopup("Workspace settings")) {
      if (ImGui::MenuItem("Restore preset layout"))
        layout = true;
      if (ImGui::MenuItem(dark ? "Use light theme" : "Use dark theme")) {
        dark = !dark;
        theme();
      }
      float scale = ImGui::GetIO().FontGlobalScale;
      if (ImGui::SliderFloat("UI scale", &scale, .85f, 1.6f))
        ImGui::GetIO().FontGlobalScale = scale;
      ImGui::Separator();
      for (auto panel :
           {"Track inspector", "Decision / Gates", "Geometry", "FOV / Depth",
            "Parameter Lab", "Experiments", "Timeline", "ImPlot metrics", "World XY history",
            "Events / Failure explorer", "A/B comparison"})
        if (ImGui::MenuItem(panel))
          selectPanel(panel);
      ImGui::EndPopup();
    }
    ImGui::SameLine();
    bufferPanel();
    ImGui::SameLine();
    ImGui::BeginDisabled(!loaded());
    if (ImGui::SmallButton("Export PNG"))
      exportRequest = "manual";
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::SmallButton("Focus view / F11"))
      toggleFocus();
    ImGui::SameLine();
    ImGui::TextDisabled("Offline | ");
    ImGui::SameLine();
    if (!error.empty()) {
      ImGui::TextColored({.8, .31, .2, 1}, "%s", error.substr(0, 90).c_str());
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", error.c_str());
    } else {
      text(status.size() > 60 ? status.substr(0, 57) + "..." : status);
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", status.c_str());
    }
    ImGui::End();
  }
  bool trackVisible(const J &t) const {
    auto &f = obj(t, "display_flags");
    if (!trackLayer)
      return false;
    if (yes(f, "actual") && !actualFilter)
      return false;
    if (yes(f, "prediction") && !predictionFilter)
      return false;
    if (yes(f, "dormant") && !dormantFilter)
      return false;
    if (yes(f, "public") ? !publicFilter : !nonpublicFilter)
      return false;
    if (yes(f, "rejected") && !rejectedFilter)
      return false;
    return !focusOnly || track == int(num(t, "id", -1));
  }
  bool clusterVisible(int cid, int side) const {
    auto &all = obj(st(side), "cluster_tags");
    auto it = all.find(std::to_string(cid));
    if (it == all.end())
      return unclassifiedFilter;
    for (auto &t : *it) {
      std::string tag = t.get<std::string>();
      if ((tag == "Matched" && !matchedFilter) ||
          (tag == "Vehicle-like" && !vehicleLikeFilter) ||
          (tag == "Unknown-like" && !unknownFilter) ||
          (tag == "Rejected components" && !rejectedClusterFilter) ||
          (tag == "Unclassified metadata" && !unclassifiedFilter))
        return false;
    }
    return true;
  }
  std::set<int> selectedClusters(int side) const {
    if (cluster >= 0)
      return {cluster};
    auto &links = obj(st(side), "verified_links");
    auto it = links.find(std::to_string(track));
    std::set<int> ids;
    if (it != links.end())
      for (auto &id : *it)
        ids.insert(id.get<int>());
    return ids;
  }
  void upload() {
    for (int side = 0; side < 2; side++) {
      auto ids = selectedClusters(side);
      std::set<int> observedIndices;
      if(cluster<0)if(const auto * tr=selected(side))
        if(yes(obj(*tr,"display_flags"),"actual"))
          for(const auto & index:arr(*tr,"current_indices"))observedIndices.insert(index.get<int>());
      std::vector<Vertex> v;
      if (pointLayer)
        for (auto &p : arr(data(), "points")) {
          if (p.size() < 10)
            continue;
          int scope = p[5].get<int>(), cid = p[4].get<int>();
          if ((scope == 2 && !foreground) || (scope == 1 && !roiPoints) ||
              (scope == 0 && !outside))
            continue;
          if (!clusterVisible(cid, side))
            continue;
          bool highlight = observedIndices.count(p[6].get<int>()) || ids.count(cid);
          if (focusOnly && !highlight)
            continue;
          v.push_back({p[0].get<float>(), p[1].get<float>(),
                       highlight    ? .38f
                       : scope == 2 ? .72f
                                    : .40f,
                       highlight    ? .94f
                       : scope == 2 ? .80f
                                    : .44f,
                       highlight    ? .61f
                       : scope == 2 ? .72f
                                    : .41f});
        }
      visiblePoints[side] = v.size();
      clouds[side].upload(v);
    }
    needsUpload = false;
  }
  bool bounds(std::array<double, 4> &b, const J &a) const {
    if (!a.is_array() || a.size() < 4)
      return false;
    for (int i = 0; i < 4; i++)
      if (!a[i].is_number())
        return false;
    for (int i = 0; i < 4; i++)
      b[i] = a[i].get<double>();
    return b[1] >= b[0] && b[3] >= b[2];
  }
  void fit(const std::array<double, 4> &b, const std::string &why) {
    camera.x = (b[0] + b[1]) / 2;
    camera.y = (b[2] + b[3]) / 2;
    double span = std::max(b[1] - b[0], (b[3] - b[2]) * 2.0);
    camera.width = std::max(2., span * 1.36 + .5);
    cameraNotice = why;
  }
  bool selectedBounds(std::array<double, 4> &b) {
    if (cluster >= 0)
      for (auto &c : arr(data(), "clusters"))
        if (int(num(c, "id")) == cluster) {
          b = {c["min"][0], c["max"][0], c["min"][1], c["max"][1]};
          return true;
        }
    const J *t = selected();
    if (!t)
      return false;
    const auto &targets = arr(*t, "camera_bounds");
    return size_t(fitTarget) < targets.size() &&
           bounds(b, arr(targets[fitTarget], "bounds"));
  }
  void fitSelected() {
    std::array<double, 4> b;
    if (selectedBounds(b)) {
      const J *t = selected();
      std::string reason = "Selected actual initial cluster; ROI excluded";
      if (cluster < 0 && t &&
          arr(*t, "camera_bounds").size() > size_t(fitTarget))
        reason = str((*t)["camera_bounds"][fitTarget], "reason");
      fit(b, reason);
    } else
      cameraNotice =
          "Selected object / requested bounds unavailable at this frame";
  }
  void fitVisible(bool roiOnly = false) {
    std::array<double, 4> b = {1e10, -1e10, 1e10, -1e10};
    auto add = [&](double x, double y) {
      b[0] = std::min(b[0], x);
      b[1] = std::max(b[1], x);
      b[2] = std::min(b[2], y);
      b[3] = std::max(b[3], y);
    };
    if (roiOnly) {
      for (auto &r : arr(data(), "roi"))
        if (yes(r, "enabled")) {
          auto &a = arr(r, "bounds");
          if (a.size() == 4) {
            add(a[0], a[2]);
            add(a[1], a[3]);
          }
          for (size_t i = 0; i < arr(r, "polygon_x").size(); i++)
            add(r["polygon_x"][i], r["polygon_y"][i]);
        }
    } else {
      auto ids = selectedClusters(anchor);
      for (auto &p : arr(data(), "points")) {
        int scope = p[5], cid = p[4];
        if (!pointLayer || (scope == 2 && !foreground) ||
            (scope == 1 && !roiPoints) || (scope == 0 && !outside) ||
            !clusterVisible(cid, anchor) || (focusOnly && !ids.count(cid)))
          continue;
        add(p[0], p[1]);
      }
      for (auto &t : arr(st(anchor), "display_tracks"))
        if (trackVisible(t)) {
          std::array<double, 4> a;
          if (bounds(a, arr(obj(t, "geometry"), "aabb"))) {
            add(a[0], a[2]);
            add(a[1], a[3]);
          } else if (arr(t, "display_position").size() == 2)
            add(t["display_position"][0], t["display_position"][1]);
        }
    }
    if (b[1] >= b[0])
      fit(b, roiOnly ? "Explicit ROI fit"
                     : "Visible rendered evidence; ROI excluded");
    else
      cameraNotice = "No visible evidence to fit";
  }
  void focusTrack(bool toggle) {
    const J *t = selected();
    if (!t || arr(*t, "display_position").size() != 2) {
      cameraNotice = "Track unavailable; previous geometry not reused";
      return;
    }
    camera.x = (*t)["display_position"][0];
    camera.y = (*t)["display_position"][1];
    if (toggle)
      follow = !follow;
    cameraNotice = str(*t, "position_source");
  }
  void layerPopup() {
    if (ImGui::BeginPopup("Layers")) {
      bool changed = false;
      for (auto p : std::vector<std::pair<const char *, bool *>>{
               {"Points", &pointLayer},
               {"Foreground", &foreground},
               {"ROI non-foreground", &roiPoints},
               {"Outside ROI", &outside},
               {"Clusters", &clusterLayer},
               {"Tracks", &trackLayer},
               {"Actual observations", &actualFilter},
               {"Prediction-only", &predictionFilter},
               {"DORMANT", &dormantFilter},
               {"Public VEHICLE", &publicFilter},
               {"Non-public", &nonpublicFilter},
               {"Rejected tracks", &rejectedFilter},
               {"Matched clusters", &matchedFilter},
               {"Vehicle-like", &vehicleLikeFilter},
               {"Unknown-like", &unknownFilter},
               {"Rejected components", &rejectedClusterFilter},
               {"Unclassified metadata", &unclassifiedFilter},
               {"Observed/stored geometry", &geometryLayer},
               {"Reliable memory", &memoryLayer},
               {"KF prediction", &kfLayer},
               {"ROI", &roiLayer},
               {"Sensor", &sensorLayer},
               {"Grid", &grid},
               {"FOV", &fovLayer}})
        changed |= ImGui::Checkbox(p.first, p.second);
      if (changed)
        needsUpload = true;
      ImGui::Separator();
      for (auto it = obj(bundle, "unsupported_filters").begin();
           it != obj(bundle, "unsupported_filters").end(); ++it) {
        ImGui::BeginDisabled();
        bool v = false;
        ImGui::Checkbox(it.key().c_str(), &v);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          ImGui::SetTooltip("%s", it.value().get<std::string>().c_str());
      }
      ImGui::SliderFloat("Point size", &pointSize, 1, 5);
      if (ImGui::Button("Show all")) {
        setLayers(J{{"points", true},
                    {"foreground", true},
                    {"roi_nonforeground", true},
                    {"outside_roi", true},
                    {"actual", true},
                    {"prediction", true},
                    {"dormant", true},
                    {"public", true},
                    {"nonpublic", true},
                    {"rejected_tracks", true},
                    {"matched", true},
                    {"vehicle_like", true},
                    {"unknown_like", true},
                    {"rejected_clusters", true},
                    {"unclassified", true}});
        focusOnly = false;
      }
      ImGui::EndPopup();
    }
  }
  void bev(int side) {
    std::string name = side ? "BEV / After" : "BEV / Before";
    if (fullFocus && !side) {
      ImGui::SetNextWindowDockID(0, ImGuiCond_Always);
      auto *vp = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos({vp->WorkPos.x, vp->WorkPos.y + 90});
      ImGui::SetNextWindowSize({vp->WorkSize.x, vp->WorkSize.y - 90});
    }
    if (!ImGui::Begin(name.c_str(), nullptr,
                      fullFocus && !side ? ImGuiWindowFlags_NoDecoration |
                                               ImGuiWindowFlags_NoDocking
                                         : 0)) {
      ImGui::End();
      return;
    }
    if (ImGui::Button("Fit selected"))
      fitSelected();
    ImGui::SameLine();
    if (ImGui::Button("Fit visible"))
      fitVisible();
    ImGui::SameLine();
    if (ImGui::Button("Fit ROI"))
      fitVisible(true);
    ImGui::SameLine();
    if (ImGui::Button("Layers"))
      ImGui::OpenPopup("Layers");
    layerPopup();
    ImGui::SameLine();
    if (ImGui::Checkbox("Only selection", &focusOnly))
      needsUpload = true;
    if (ImGui::Button(follow ? "Following" : "Follow track")) {
      anchor = side;
      focusTrack(true);
    }
    ImGui::SameLine();
    if (ImGui::Button("Focus track")) {
      anchor = side;
      focusTrack(false);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(170);
    ImGui::Combo("##fit", &fitTarget,
                 "Observed cluster\0Current geometry\0Reliable geometry\0Track "
                 "+ prediction\0");
    ImGui::SameLine();
    ImGui::TextDisabled("World XY | %.2f m wide", camera.width);
    ImVec2 size = ImGui::GetContentRegionAvail();
    size.y = std::max(70.f, size.y - 42);
    size.x = std::max(100.f, size.x);
    ImVec2 origin = ImGui::GetCursorScreenPos();
    if (needsUpload)
      upload();
    ImGuiIO &io = ImGui::GetIO();
    float dpi = io.DisplayFramebufferScale.x;
    GLuint tex = clouds[side].render(int(size.x * dpi), int(size.y * dpi),
                                     camera, pointSize * dpi);
    ImGui::Image((ImTextureID)(intptr_t)tex, size, {0, 1}, {1, 0});
    bool hover = ImGui::IsItemHovered();
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(origin, {origin.x + size.x, origin.y + size.y}, true);
    auto point = [&](double x, double y) {
      return ImVec2(origin.x + size.x * .5f +
                        float((x - camera.x) * size.x / camera.width),
                    origin.y + size.y * .5f -
                        float((y - camera.y) * size.x / camera.width));
    };
    auto line = [&](double x, double y, double x2, double y2, ImU32 c,
                    bool dash = false, float width = 1.f) {
      auto a = point(x, y), b = point(x2, y2);
      if (!dash)
        draw->AddLine(a, b, c, width);
      else {
        float dx = b.x - a.x, dy = b.y - a.y,
              len = std::sqrt(dx * dx + dy * dy);
        for (float t = 0; t < len; t += 10) {
          float u = std::min(t + 5, len);
          draw->AddLine({a.x + dx * t / std::max(len, .01f),
                         a.y + dy * t / std::max(len, .01f)},
                        {a.x + dx * u / std::max(len, .01f),
                         a.y + dy * u / std::max(len, .01f)},
                        c, width);
        }
      }
    };
    auto box = [&](const J &a, ImU32 c, bool dash) {
      std::array<double, 4> b;
      if (!bounds(b, a))
        return;
      line(b[0], b[2], b[1], b[2], c, dash);
      line(b[1], b[2], b[1], b[3], c, dash);
      line(b[1], b[3], b[0], b[3], c, dash);
      line(b[0], b[3], b[0], b[2], c, dash);
    };
    if (grid) {
      double step = std::pow(10., std::floor(std::log10(camera.width / 8)));
      if (camera.width / step > 18)
        step *= 2;
      double halfY = camera.width * size.y / size.x / 2;
      for (double x = std::floor((camera.x - camera.width / 2) / step) * step;
           x < camera.x + camera.width / 2; x += step)
        line(x, camera.y - halfY, x, camera.y + halfY, col(.35, .4, .35, .22));
      for (double y = std::floor((camera.y - halfY) / step) * step;
           y < camera.y + halfY; y += step)
        line(camera.x - camera.width / 2, y, camera.x + camera.width / 2, y,
             col(.35, .4, .35, .22));
    }
    if (roiLayer)
      for (auto &r : arr(data(), "roi")) {
        if (!yes(r, "enabled"))
          continue;
        auto &xs = arr(r, "polygon_x");
        auto &ys = arr(r, "polygon_y");
        if (xs.size() > 1 && xs.size() == ys.size())
          for (size_t i = 0; i < xs.size(); i++)
            line(xs[i], ys[i], xs[(i + 1) % xs.size()], ys[(i + 1) % ys.size()],
                 col(.69, .62, .45, .6), true);
        else
          box(arr(r, "bounds"), col(.69, .62, .45, .6), true);
      }
    if (sensorLayer && arr(data(), "sensor_origin").size() == 3) {
      auto &a = data()["sensor_origin"];
      auto q = point(a[0], a[1]);
      draw->AddCircle(q, 5, col(.9, .83, .55));
      draw->AddText({q.x + 8, q.y}, col(.9, .83, .55), "LiDAR");
    }
    auto ids = selectedClusters(side);
    if (clusterLayer)
      for (auto &c : arr(data(), "clusters")) {
        int id = int(num(c, "id"));
        if (!clusterVisible(id, side) || (focusOnly && !ids.count(id)))
          continue;
        J a = J::array({c["min"][0], c["max"][0], c["min"][1], c["max"][1]});
        auto color = ids.count(id) ? col(.4, .95, .6) : col(.57, .65, .59, .6);
        box(a, color, false);
        auto q = point(c["center"][0], c["center"][1]);
        if (ids.count(id) || camera.width < 35) {
          std::string s = "C" + std::to_string(id) + " / " + str(c, "count");
          draw->AddText({q.x + 5, q.y + 5}, color, s.c_str());
        }
      }
    for (auto &t : arr(st(side), "display_tracks")) {
      if (!trackVisible(t))
        continue;
      bool actual = yes(obj(t, "display_flags"), "actual");
      bool selected = int(num(t, "id")) == track;
      ImU32 color = selected ? col(.42, .95, .62)
                    : actual ? col(.72, .85, .73)
                             : col(.77, .6, .86);
      auto &pos = arr(t, "display_position");
      if (pos.size() != 2)
        continue;
      auto q = point(pos[0], pos[1]);
      draw->AddCircle(q, selected ? 6 : 3, color, 0, selected ? 2 : 1);
      std::string title =
          "T" + str(t, "id") + " " + (actual ? "observed" : "prediction");
      if (selected || camera.width < 60)
        draw->AddText({q.x + 8, q.y - 16}, color, title.c_str());
      auto &g = obj(t, "geometry");
      if (geometryLayer)
        box(arr(g, "aabb"), color, !actual);
      if (memoryLayer && num(obj(t, "memory"), "last_reliable_stamp") > 0)
        box(arr(g, "reliable_aabb"), col(.8, .64, .35), true);
      if (kfLayer) {
        auto &p = arr(obj(t, "observation"), "predicted");
        if (p.size() == 2) {
          auto a = point(p[0], p[1]);
          draw->AddLine({a.x - 5, a.y - 5}, {a.x + 5, a.y + 5},
                        col(.8, .6, .92), 1.5);
          draw->AddLine({a.x - 5, a.y + 5}, {a.x + 5, a.y - 5},
                        col(.8, .6, .92), 1.5);
        }
      }
      if (fovLayer) {
        auto &v = obj(t, "visibility");
        if (selected && v.contains("range")) {
          char range[48];
          snprintf(range, sizeof(range), "r=%.2f m | ", num(v, "range"));
          std::string s = std::string(range) + str(v, "reason");
          draw->AddText({q.x + 8, q.y + 10}, col(.8, .8, .68), s.c_str());
        }
      }
    }
    draw->PopClipRect();
    if (hover) {
      anchor = side;
      if (io.MouseWheel) {
        double mx = camera.x + (io.MousePos.x - origin.x - size.x / 2) *
                                   camera.width / size.x,
               my = camera.y - (io.MousePos.y - origin.y - size.y / 2) *
                                   camera.width / size.x;
        double old = camera.width;
        camera.width =
            std::clamp(camera.width * std::pow(.82, io.MouseWheel), .2, 2000.);
        camera.x = mx - (mx - camera.x) * camera.width / old;
        camera.y = my - (my - camera.y) * camera.width / old;
      }
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) ||
          ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        camera.x -= io.MouseDelta.x * camera.width / size.x;
        camera.y += io.MouseDelta.y * camera.width / size.x;
        follow = false;
      }
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        float best = 14;
        int tid = -1;
        for (auto &t : arr(st(side), "display_tracks")) {
          auto &p = arr(t, "display_position");
          if (p.size() != 2 || !trackVisible(t))
            continue;
          auto q = point(p[0], p[1]);
          float d = std::hypot(q.x - io.MousePos.x, q.y - io.MousePos.y);
          if (d < best) {
            best = d;
            tid = int(num(t, "id"));
          }
        }
        if (tid >= 0)
          pickTrack(tid);
        else {
          int cid = -1;
          best = 12;
          for (auto &p : arr(data(), "points")) {
            int id = p[4], scope = p[5];
            if (!pointLayer || id < 0 || !clusterVisible(id, side) ||
                (scope == 2 && !foreground) || (scope == 1 && !roiPoints) ||
                (scope == 0 && !outside) || (focusOnly && !ids.count(id)))
              continue;
            auto q = point(p[0], p[1]);
            float d = std::hypot(q.x - io.MousePos.x, q.y - io.MousePos.y);
            if (d < best) {
              best = d;
              cid = id;
            }
          }
          if (cid >= 0) {
            cluster = cid;
            track = -1;
            graphs[0] = graphs[1] = J::object();
            follow = false;
            needsUpload = true;
          }
        }
      }
      if (!ImGui::IsMouseDragging(ImGuiMouseButton_Right) &&
          !ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        int bestIndex = -1;
        float best = 5;
        for (size_t i = 0; i < arr(data(), "points").size(); i++) {
          auto &p = data()["points"][i];
          int scope = p[5], cid = p[4];
          if (!pointLayer || (scope == 2 && !foreground) ||
              (scope == 1 && !roiPoints) || (scope == 0 && !outside) ||
              !clusterVisible(cid, side) || (focusOnly && !ids.count(cid)))
            continue;
          auto q = point(p[0], p[1]);
          float d = std::hypot(q.x - io.MousePos.x, q.y - io.MousePos.y);
          if (d < best) {
            best = d;
            bestIndex = int(i);
          }
        }
        if (bestIndex >= 0) {
          auto &p = data()["points"][bestIndex];
          ImGui::BeginTooltip();
          ImGui::Text("Prepared compact point index %d / C%d", p[6].get<int>(),
                      p[4].get<int>());
          ImGui::Text("World %.3f, %.3f, %.3f m | ring %d", p[0].get<double>(),
                      p[1].get<double>(), p[2].get<double>(), p[3].get<int>());
          ImGui::Text("Range %.3f m", p[9].get<double>());
          ImGui::EndTooltip();
        }
      }
    }
    ImGui::TextDisabled("%d actual / %d shown / %d LOD points | %d clusters",
                        int(num(data(), "actual_points")), visiblePoints[side],
                        int(num(data(), "rendered_points")),
                        int(arr(data(), "clusters").size()));
    ImGui::TextDisabled("%s", cameraNotice.c_str());
    ImGui::End();
  }
  void runCombo(int side) {
    std::string title = side ? "After" : "Before";
    ImGui::SetNextItemWidth(280);
    if (ImGui::BeginCombo(title.c_str(), runLabels[side].empty()
                                             ? "No result"
                                             : runLabels[side].c_str())) {
      for (auto &r : arr(opened, "runs"))
        if (ImGui::Selectable(str(r, "label").c_str(),
                              runs[side] == str(r, "id"))) {
          runs[side] = str(r, "id");
          whatif = J::object();
          runLabels[side] = str(r, "label");
          requestedFrame = -1;
          fetch();
          loadAnalysis();
        }
      ImGui::EndCombo();
    }
  }
  void graphSeek(int value, int side = -1) {
    playing = false;
    if(side >= 0) anchor = side;
    kfLayer = memoryLayer = geometryLayer = true;
    fitGraphPending = true; graphClickFrame = std::clamp(value,1,total);
    seek(value);
    if(loaded() && requestedFrame == frame && num(data(),"frame",-1)==frame) {
      fitSelected(); fitGraphPending=false;
    }
  }
  static bool recorded(const J & row, const char * key) {
    auto p=row.find(key);return p!=row.end()&&p->is_number()&&std::isfinite(p->get<double>());
  }
  static void historyLine(const std::string & label,const J & rows,
                          const char * xkey,const char * ykey,ImVec4 color,
                          const char * mask=nullptr,bool dashed=false) {
    std::vector<double> x,y;int previous=-1;
    const double gap=std::numeric_limits<double>::quiet_NaN();
    for(const auto & row:rows) {
      int f=int(num(row,"frame"));
      if(previous>=0&&f!=previous+1){x.push_back(gap);y.push_back(gap);}
      bool valid=recorded(row,xkey)&&recorded(row,ykey)&&(!mask||yes(row,mask));
      x.push_back(valid?num(row,xkey):gap);y.push_back(valid?num(row,ykey):gap);previous=f;
    }
    if(x.empty())return;
    ImPlot::SetNextLineStyle(color,1.7f);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle,1.8f,color,1,color);
    // Default ImPlot NaN handling breaks lines; never enable SkipNaN here.
    ImPlot::PlotLine(label.c_str(),x.data(),y.data(),int(x.size()),
                    dashed?ImPlotLineFlags_Segments:ImPlotLineFlags_None);
  }
  void graphControls() {
    if(ImGui::Checkbox("Use frame range",&graphRange)) {
      graphMin=graphRange?rangeStart:1;graphMax=graphRange?std::max(rangeStart+1,rangeEnd):std::max(2,total);
    }
    ImGui::SameLine();
    if(ImGui::SmallButton("Center on current")) {
      graphMin=std::max(1,frame-50);graphMax=std::min(total,frame+50);graphRange=false;
    }
    ImGui::SameLine();
    if(ImGui::SmallButton("All frames")){graphMin=1;graphMax=std::max(2,total);graphRange=false;}
    ImGui::SameLine();ImGui::TextDisabled("Track %d | Before charcoal / After green | gaps stay empty",track);
  }
  void frameGraphInteraction(bool lanes=false,const char * metricField=nullptr) {
    double here=frame;
    ImPlot::DragLineX(801,&here,{.4f,.7f,.48f,.7f},1,ImPlotDragToolFlags_NoInputs);
    if(int(std::round(here))!=frame)graphSeek(int(std::round(here)));
    if(ImPlot::IsPlotHovered()&&ImGui::IsMouseReleased(0)&&std::hypot(ImGui::GetMouseDragDelta(0).x,ImGui::GetMouseDragDelta(0).y)<3)
      {
        const auto mouse=ImPlot::GetPlotMousePos();int side=-1;
        if(lanes)side=(mouse.y-std::round(mouse.y))>0?1:0;
        if(metricField) {
          double best=1e30;ImVec2 pixels=ImGui::GetMousePos();
          for(int i=0;i<2;++i)for(const auto & q:arr(graphs[i],"samples"))
            if(int(num(q,"frame"))==int(std::round(mouse.x))&&recorded(q,metricField)) {
              ImVec2 p=ImPlot::PlotToPixels(num(q,"frame"),num(q,metricField));
              double d=std::hypot(p.x-pixels.x,p.y-pixels.y);if(d<best){best=d;side=i;}
            }
        }
        graphSeek(int(std::round(mouse.x)),side);
      }
    if(ImPlot::IsPlotHovered()) {
      int f=int(std::round(ImPlot::GetPlotMousePos().x));
      ImGui::BeginTooltip();ImGui::Text("Frame %d / Track %d; click to inspect current C++ evidence",f,track);
      for(int side=0;side<2;++side)for(const auto & row:arr(graphs[side],"samples"))if(int(num(row,"frame"))==f) {
        ImGui::Text("%s: actual=%s public=%s prediction-only=%s",side?"After":"Before",
                    str(row,"actual","Not recorded").c_str(),str(row,"vehicle_public","Not recorded").c_str(),str(row,"prediction_only","Not recorded").c_str());
        ImGui::Text("Bins %s / Required %s (%s)",str(row,"angular_bins","Not recorded").c_str(),
                    str(row,"required_bins","Not recorded").c_str(),str(row,"angular_scope","No C++ angular evaluation").c_str());
      }
      ImGui::EndTooltip();
    }
  }
  void timeline() {
    if (!ImGui::Begin("Timeline")) {
      ImGui::End();
      return;
    }
    runCombo(0);
    ImGui::SameLine();
    runCombo(1);
    ImGui::SameLine();
    const char *anchors[] = {"Before camera / selection",
                             "After camera / selection"};
    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("##anchor", &anchor, anchors, 2))
      loadAnalysis();
    if (ImGui::Button(playing ? "Pause" : "Play"))
      playing = !playing;
    ImGui::SameLine();
    if (ImGui::Button("<")) {
      playing = false;
      seek(frame - 1);
    }
    ImGui::SameLine();
    if (ImGui::Button(">")) {
      playing = false;
      seek(frame + 1);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::SliderFloat("Speed", &speed, .25f, 4, "%.2fx");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(
        std::max(150.f, ImGui::GetContentRegionAvail().x - 160));
    int v = frame;
    if (ImGui::SliderInt("Frame", &v, 1, total))
      seek(v);
    ImGui::SameLine();
    ImGui::Text("/ %d", total);
    double sec = 0;
    if (frame > 0 && size_t(frame) <= arr(opened, "rows").size())
      sec = num(opened["rows"][frame - 1], "relative_sec");
    ImGui::SetNextItemWidth(140);
    if (ImGui::InputDouble("Time (s)", &sec, 0, 0, "%.3f",
                           ImGuiInputTextFlags_EnterReturnsTrue)) {
      int n = 1;
      double closest = 1e20;
      for (auto &r : arr(opened, "rows")) {
        double d = std::abs(num(r, "relative_sec") - sec);
        if (d < closest) {
          closest = d;
          n = int(num(r, "frame"));
        }
      }
      seek(n);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(145);
    ImGui::InputInt("Detail start", &rangeStart);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(145);
    ImGui::InputInt("End", &rangeEnd);
    ImGui::SameLine();
    if (ImGui::Button("Use current frame"))
      rangeStart = rangeEnd = frame;
    graphControls();
    if (ImPlot::BeginPlot("Recorded tracking lines",{-1,std::max(140.f,ImGui::GetContentRegionAvail().y-24)})) {
      ImPlot::SetupAxes("Frame","Recorded lanes",0,ImPlotAxisFlags_Lock);
      ImPlot::SetupAxisLinks(ImAxis_X1,&graphMin,&graphMax);
      ImPlot::SetupAxisLimits(ImAxis_Y1,-.5,8.5,ImGuiCond_Always);
      const char * names[]={"Actual points","Cluster matched","Public VEHICLE","Prediction-only",
          "Canonical LOST","DORMANT","Unlinked bounded pieces","Class not VEHICLE","KF available"};
      const char * keys[]={"actual","cluster_matched","vehicle_public","prediction_only","lost",
          "dormant","unmatched_cluster","classification_rejected","kf_available"};
      double ticks[]={0,1,2,3,4,5,6,7,8};ImPlot::SetupAxisTicks(ImAxis_Y1,ticks,9,names);
      for(int side=0;side<2;++side)for(int lane=0;lane<9;++lane) {
        std::vector<double>x,y;int prev=-1;const double gap=std::numeric_limits<double>::quiet_NaN();
        for(const auto & row:arr(graphs[side],"samples")) {
          int f=int(num(row,"frame"));if(prev>=0&&f!=prev+1){x.push_back(gap);y.push_back(gap);}
          x.push_back(f);y.push_back(yes(row,keys[lane])?lane+(side?.13:-.13):gap);prev=f;
        }
        if(!x.empty()) {
          ImVec4 c=side?ImVec4(.2f,.65f,.43f,1):ImVec4(.35f,.35f,.34f,1);
          ImPlot::SetNextLineStyle(c,2);ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle,1.5f,c,1,c);
          std::string label=std::string(side?"After ":"Before ")+names[lane];
          ImPlot::PlotLine(label.c_str(),x.data(),y.data(),int(x.size()),ImPlotItemFlags_NoLegend);
        }
      }
      std::map<int,const J*> before;
      for(const auto & row:arr(graphs[0],"samples"))before[int(num(row,"frame"))]=&row;
      for(int lane:{0,2})for(bool lost:{false,true}) {
        std::vector<double>x,y;const char * key=lane?"vehicle_public":"actual";
        for(const auto & row:arr(graphs[1],"samples")) {
          int f=int(num(row,"frame"));auto it=before.find(f);if(it==before.end())continue;
          bool av=yes(*it->second,key),bv=yes(row,key);
          if(lost?(av&&!bv):(!av&&bv)){x.push_back(f);y.push_back(lane+.28);}
        }
        if(!x.empty()) {
          ImPlot::SetNextMarkerStyle(lost?ImPlotMarker_Cross:ImPlotMarker_Up,4,
              lost?ImVec4(.85f,.3f,.2f,1):ImVec4(.1f,.65f,.35f,1));
          ImPlot::PlotScatter(lost?"Lost observations":"Added observations",x.data(),y.data(),int(x.size()));
        }
      }
      if(validateTrackingLines)validationTimelinePoint=ImPlot::PlotToPixels(validationTargetFrame,2.13);
      frameGraphInteraction(true);ImPlot::EndPlot();
    }
    ImGui::TextDisabled("Actual and Cluster matched share the recorded point-association contract. Empty canonical/unlinked data means not recorded.");
    ImGui::End();
  }
  void trackInspector() {
    if (!ImGui::Begin("Track inspector")) {
      ImGui::End();
      return;
    }
    std::string caption =
        track < 0 ? (cluster < 0 ? "Choose object"
                                 : "Cluster " + std::to_string(cluster))
                  : "Track " + std::to_string(track);
    if (ImGui::BeginCombo("Selection", caption.c_str())) {
      if (ImGui::Selectable("All tracks", track < 0 && cluster < 0))
        pickTrack(-1);
      for (auto &t : arr(st(anchor), "display_tracks"))
        if (ImGui::Selectable(("Track " + str(t, "id")).c_str(),
                              int(num(t, "id")) == track))
          pickTrack(int(num(t, "id")));
      for (auto &c : arr(data(), "clusters"))
        if (ImGui::Selectable(("Cluster " + str(c, "id")).c_str(),
                              int(num(c, "id")) == cluster)) {
          cluster = int(num(c, "id"));
          track = -1;
          graphs[0] = graphs[1] = J::object();
          follow = false;
          needsUpload = true;
        }
      ImGui::EndCombo();
    }
    const J *t = selected();
    if (!t) {
      wrapped(cluster >= 0
                  ? "Selected actual initial cluster. Track assignment is "
                    "shown only when verified by stored evidence."
                  : "Selected track unavailable at this frame. No stale "
                    "geometry is substituted.");
      for (auto &c : arr(data(), "clusters"))
        if (int(num(c, "id")) == cluster)
          raw("Actual cluster", c);
      ImGui::End();
      return;
    }
    ImGui::SetWindowFontScale(1.3f);
    text("Track " + str(*t, "id"));
    ImGui::SetWindowFontScale(1);
    text(str(*t, "lifecycle", "Lifecycle not recorded") + " / " +
         str(*t, "status"));
    bool actual = yes(obj(*t, "display_flags"), "actual");
    ImGui::TextColored(
        actual ? ImVec4(.25, .65, .4, 1) : ImVec4(.75, .55, .85, 1), "%s",
        actual ? "Actual observation" : "Prediction / missed observation");
    text(str(*t, "position_source"));
    ImGui::Separator();
    wrapped("Detection: " + str(*t, "decision_reason", "Not recorded"));
    text("Output class score: " + brief(*t, "score", "Not recorded"));
    if(t->contains("vehicle_size_score"))text("Vehicle size score: " + brief(*t,"vehicle_size_score") + " / " + str(*t,"vehicle_score_threshold","0.65"));
    text("Public VEHICLE: " +
         std::string(yes(obj(*t, "display_flags"), "public") ? "yes" : "no"));
    text("Raw association: " +
         str(*t, "association_source", str(*t, "association", "Not recorded")));
    wrapped(
        "Raw association can disagree with the observation flag. A detection "
        "gate failure is not automatically the final association cause.");
    auto &k = obj(*t, "kf");
    const auto &state = arr(k, "state");
    if (state.size() >= 4)
      ImGui::Text("KF XY %.3f, %.3f m\nVelocity %.3f, %.3f m/s",
                  state[0].get<double>(), state[1].get<double>(),
                  state[2].get<double>(), state[3].get<double>());
    else
      wrapped("KF state: " + state.dump());
    text("Lost frames: " +
         str(*t, "lost_frames", str(*t, "lost", "Not recorded")));
    if (ImGui::Button("Fit this object"))
      fitSelected();
    ImGui::SameLine();
    if (ImGui::Button("Follow"))
      focusTrack(true);
    raw("Advanced recorded values", *t);
    ImGui::End();
  }
  std::string gateSource(const J &gate) {
    for (auto &c : arr(st(anchor), "gate_catalog"))
      if (str(c, "function") == str(gate, "function") &&
          str(c, "gate") == str(gate, "gate"))
        return str(c, "source");
    return str(gate, "source", "Source unavailable in this trace");
  }
  void gatesTable(const J &gates) {
    if (ImGui::BeginTable(
            "gate-values", 4,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX,
            {0, 0})) {
      for (auto h : {"Gate / Function", "Value", "Threshold", "Result"})
        ImGui::TableSetupColumn(h);
      ImGui::TableHeadersRow();
      for (auto &g : gates) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        text(str(g, "gate"));
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("%s\n%s:%s\nTrack %s / Cluster %s",
                            str(g, "function").c_str(), gateSource(g).c_str(),
                            str(g, "line").c_str(), str(g, "track").c_str(),
                            str(g, "cluster").c_str());
        ImGui::TableNextColumn();
        text(brief(g, "value"));
        ImGui::TableNextColumn();
        text(brief(g, "threshold"));
        ImGui::TableNextColumn();
        std::string status = str(g, "status", yes(g, "pass") ? "PASS" : "FAIL");
        if (g.contains("passed"))
          status = yes(g, "passed") ? "PASS" : "FAIL";
        if (g.contains("evaluated") && !yes(g, "evaluated"))
          status = "NOT_EVALUATED";
        ImGui::TextColored(status == "FAIL"   ? ImVec4(.82, .35, .23, 1)
                           : status == "PASS" ? ImVec4(.25, .65, .4, 1)
                                              : ImVec4(.55, .56, .53, 1),
                           "%s", status.c_str());
      }
      ImGui::EndTable();
    }
  }
  void gates() {
    if (!ImGui::Begin("Decision / Gates")) {
      ImGui::End();
      return;
    }
    const J *t = selected();
    bool present = t && !arr(*t, "gate_inspector").empty();
    if (!whatif.empty()) {
      wrapped(str(whatif, "scope"));
      for (auto &w : arr(whatif, "tracks"))
        if (int(num(w, "id")) == track) {
          raw("Baseline decision", obj(w, "baseline"));
          raw("Candidate decision", obj(w, "candidate"));
          if (ImGui::CollapsingHeader("Baseline frozen gates")) {
            ImGui::PushID("baseline");
            gatesTable(arr(w, "baseline_gates"));
            ImGui::PopID();
          }
          if (ImGui::CollapsingHeader("Candidate frozen gates",
                                      ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::PushID("candidate");
            gatesTable(arr(w, "candidate_gates"));
            ImGui::PopID();
          }
        }
      if (ImGui::Button("Return to recorded trace"))
        whatif = J::object();
    } else if (present) {
      const J &gs = arr(*t, "gate_inspector");
      for (auto &g : gs)
        if ((g.contains("evaluated") ? yes(g, "evaluated") : true) &&
            g.contains("pass") && !yes(g, "pass")) {
          ImGui::TextColored({.85, .36, .23, 1}, "%s", str(g, "gate").c_str());
          text(brief(g, "value") + " / " + brief(g, "threshold"));
        }
      gatesTable(gs);
      for (auto &call : arr(*t, "gate_calls")) {
        std::string title = str(call, "function") + " / C" +
                            str(call, "cluster") + " / call " +
                            str(call, "call");
        if (ImGui::TreeNode(title.c_str())) {
          gatesTable(arr(call, "gates"));
          ImGui::TreePop();
        }
      }
    } else
      wrapped(
          "No detailed C++ gates recorded for this run/frame/track. Use a "
          "compatible Detailed Replay; missing gate causes are not inferred.");
    wrapped("NOT_EVALUATED means an earlier return prevented this condition "
            "from running. This is not a final Association verdict.");
    ImGui::End();
  }
  void geometry() {
    if (!ImGui::Begin("Geometry")) {
      ImGui::End();
      return;
    }
    const J *t = selected();
    if (t) {
      auto &g = obj(*t, "geometry");
      for (auto key : {"length", "width", "height", "confidence", "update",
                       "reliable_size"})
        text(std::string(key) + ": " + str(g, key, "Not recorded"));
      raw("Current / memory geometry", g);
      raw("KF covariance and innovation", obj(*t, "kf"));
      raw("Tracking memory", obj(*t, "memory"));
    } else
      wrapped("Choose a recorded track. Memory is shown separately from actual "
              "points.");
    ImGui::End();
  }
  void fov() {
    if (!ImGui::Begin("FOV / Depth")) {
      ImGui::End();
      return;
    }
    const J *t = selected();
    if (t) {
      auto &v = obj(*t, "visibility");
      text("Visibility: " + str(v, "reason", "Not recorded"));
      ImGui::Text("Range %.2f m | Expected %s / Supported %s rays",
                  num(v, "range"), str(v, "expected_rays", "N/A").c_str(),
                  str(v, "supported_rays", "N/A").c_str());
      ImGui::Text("Supported %.3f | Occluded %.3f",
                  num(v, "supported_fraction"), num(v, "occluded_fraction"));
    }
    ImGui::Text("%d stored depth cells | Ray spacing %.4f deg",
                int(num(data(), "depth_count")),
                num(obj(data(), "index"), "ray_elevation_spacing_deg"));
    if (ImPlot::BeginPlot("Stored actual depth", {-1, 180})) {
      ImPlot::SetupAxes("Azimuth (deg)", "Elevation (deg)",
                        ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      for (int band = 0; band < 4; band++) {
        std::vector<double> x, y;
        for (auto &p : arr(data(), "depth")) {
          double range = p[2];
          int category = range < 15 ? 0 : range < 35 ? 1 : range < 60 ? 2 : 3;
          if (category == band) {
            x.push_back(p[0]);
            y.push_back(p[1]);
          }
        }
        const char *labels[] = {"<15 m", "15-35 m", "35-60 m", ">=60 m"};
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.5);
        if (!x.empty())
          ImPlot::PlotScatter(labels[band], x.data(), y.data(), x.size());
      }
      ImPlot::EndPlot();
    }
    wrapped("Points = stored returns. Expected rays / boxes = model geometry.");
    if (t)
      raw("Recorded FOV / visibility values", obj(*t, "visibility"));
    ImGui::End();
  }
  void metrics() {
    if (!ImGui::Begin("ImPlot metrics")) {
      ImGui::End();
      return;
    }
    std::set<std::string> available;
    for (auto &g : graphs)
      for (auto &s : arr(g, "samples"))
        for (auto it = s.begin(); it != s.end(); ++it)
          if (it.key() != "frame" && it.key() != "sec" &&
              it.value().is_number())
            available.insert(it.key());
    for (auto &g : graphs)
      if (!arr(g, "timing").empty())
        available.insert("pipeline_ms");
    std::vector<std::string> names(available.begin(), available.end());
    if (names.empty()) {
      wrapped("No metric series recorded for this selection. Select a Track "
              "and a completed Run.");
      ImGui::End();
      return;
    }
    auto found = std::find(names.begin(), names.end(), metricKey);
    metric = found == names.end() ? 0 : int(found - names.begin());
    ImGui::SetNextItemWidth(240);
    if (ImGui::BeginCombo("Metric", names[metric].c_str())) {
      for (size_t i = 0; i < names.size(); i++)
        if (ImGui::Selectable(names[i].c_str(), int(i) == metric))
          metric = int(i), metricKey = names[i];
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Run-local Track IDs; missing samples remain gaps");
    graphControls();
    if (ImPlot::BeginPlot(
            "Before / After",
            {-1, std::max(100.f, ImGui::GetContentRegionAvail().y)})) {
      ImPlot::SetupAxes("Frame", names[metric].c_str(), 0,ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLinks(ImAxis_X1,&graphMin,&graphMax);
      for (int side = 0; side < 2; side++) {
        auto &rows = arr(graphs[side],
                         names[metric] == "pipeline_ms" ? "timing" : "samples");
        std::vector<double> x, y;
        for (auto &s : rows)
          if (s.contains(names[metric]) && s[names[metric]].is_number()) {
            x.push_back(num(s, "frame"));
            y.push_back(s[names[metric]].get<double>());
          }
        if (!x.empty()) {
          ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2);
          // Rebuild with explicit NaNs for absent/nonconsecutive samples.
          historyLine(side?"After":"Before",rows,"frame",names[metric].c_str(),
                      side?ImVec4(.2f,.65f,.43f,1):ImVec4(.35f,.35f,.34f,1));
          const char * threshold=names[metric]=="angular_bins"?"required_bins":
              names[metric]=="vehicle_size_score"?"vehicle_score_threshold":
              names[metric]=="association_distance"?"association_spatial_gate":nullptr;
          if(threshold)historyLine(side?"After required":"Before required",rows,"frame",threshold,
                      side?ImVec4(.65f,.55f,.25f,1):ImVec4(.6f,.55f,.5f,1));
        }
      }
      frameGraphInteraction(false,names[metric].c_str());
      ImPlot::EndPlot();
    }
    ImGui::End();
  }
  void trajectory() {
    if(!ImGui::Begin("World XY history")){ImGui::End();return;}
    ImGui::Checkbox("World 50m evaluation boundary (not ROI)",&trajectoryBoundary);
    ImGui::SameLine();
    if(ImGui::SmallButton("Fit selected"))fitSelected();
    ImGui::SameLine();
    if(ImGui::SmallButton("Focus Track"))focusTrack(false);
    ImGui::SameLine();
    if(ImGui::SmallButton(follow?"Following":"Follow Track"))focusTrack(true);
    wrapped("Solid: actual C++ measurement. Broken model segments: recorded KF prior. No measurement line crosses a missing frame. IDs are run-local.");
    if(trajectoryFit){ImPlot::SetNextAxesToFit();trajectoryFit=false;}
    if(ImPlot::BeginPlot("Measurement / KF history",{-1,std::max(180.f,ImGui::GetContentRegionAvail().y)})) {
      ImPlot::SetupAxes("World X (m)","World Y (m)");
      for(int side=0;side<2;++side) {
        J rows=J::array();for(const auto & q:arr(graphs[side],"samples"))
          if(num(q,"frame")>=graphMin&&num(q,"frame")<=graphMax)rows.push_back(q);
        if(validateTrackingLines&&side==1)for(const auto & q:rows)
          if(int(num(q,"frame"))==validationTargetFrame&&recorded(q,"measurement_x")&&recorded(q,"measurement_y"))
            validationTrajectoryPoint=ImPlot::PlotToPixels(num(q,"measurement_x"),num(q,"measurement_y"));
        historyLine(side?"After actual measurement":"Before actual measurement",rows,"measurement_x","measurement_y",
                    side?ImVec4(.2f,.65f,.43f,1):ImVec4(.35f,.35f,.34f,1),"actual");
        historyLine(side?"After KF prior (model)":"Before KF prior (model)",rows,"prediction_x","prediction_y",
                    side?ImVec4(.6f,.37f,.7f,1):ImVec4(.65f,.58f,.68f,1),nullptr,true);
        std::set<int> resumed;
        for(const auto & e:arr(graphs[side],"events"))if(str(e,"kind")=="reacquired"||str(e,"kind")=="observed_again")resumed.insert(int(num(e,"frame")));
        std::vector<double>x,y;
        for(const auto & row:rows)if(resumed.count(int(num(row,"frame")))&&recorded(row,"measurement_x")&&recorded(row,"measurement_y")){
          x.push_back(num(row,"measurement_x"));y.push_back(num(row,"measurement_y"));}
        if(!x.empty()){ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond,4);
          ImPlot::PlotScatter(side?"After observation resumed":"Before observation resumed",x.data(),y.data(),int(x.size()));}
      }
      if(trajectoryBoundary) {
        std::vector<double>x,y;for(int i=0;i<=180;++i){double a=i*2*3.141592653589793/180;x.push_back(50*std::cos(a));y.push_back(50*std::sin(a));}
        ImPlot::SetNextLineStyle({.7f,.65f,.45f,.7f},1);
        ImPlot::PlotLine("World 50m (evaluation only)",x.data(),y.data(),int(x.size()),ImPlotItemFlags_NoFit);
      }
      if(ImPlot::IsPlotHovered()) {
        ImVec2 mouse=ImGui::GetMousePos();double best=14*14;int selectedFrame=-1,selectedSide=-1;
        for(int side=0;side<2;++side)for(const auto & q:arr(graphs[side],"samples")) {
          if(num(q,"frame")<graphMin||num(q,"frame")>graphMax)continue;
          for(const auto & keys:std::array<std::pair<const char*,const char*>,2>{{{"measurement_x","measurement_y"},{"prediction_x","prediction_y"}}}) {
            if(!recorded(q,keys.first)||!recorded(q,keys.second))continue;
            ImVec2 p=ImPlot::PlotToPixels(num(q,keys.first),num(q,keys.second));
            double d=(p.x-mouse.x)*(p.x-mouse.x)+(p.y-mouse.y)*(p.y-mouse.y);
            if(d<best){best=d;selectedFrame=int(num(q,"frame"));selectedSide=side;}
          }
        }
        if(selectedFrame>=0){ImGui::BeginTooltip();ImGui::Text("%s / Track %d / Frame %d; click for BEV and gates",selectedSide?"After":"Before",track,selectedFrame);ImGui::EndTooltip();
          if(ImGui::IsMouseReleased(0)&&std::hypot(ImGui::GetMouseDragDelta(0).x,ImGui::GetMouseDragDelta(0).y)<3)graphSeek(selectedFrame,selectedSide);}
      }
      ImPlot::EndPlot();
    }
    ImGui::End();
  }
  void eventPanel() {
    if (!ImGui::Begin("Events / Failure explorer")) {
      ImGui::End();
      return;
    }
    ImGui::InputText("Filter", eventSearch, sizeof(eventSearch));
    ImGui::SameLine();
    if (ImGui::Button("Refresh"))
      loadAnalysis();
    ImGui::SameLine();
    static bool selectedOnly = false;
    ImGui::Checkbox("Selected track only", &selectedOnly);
    wrapped("Only recorded events. Canonical LOST is unavailable if the trace "
            "lacks its lifecycle enum.");
    if (ImGui::BeginTable(
            "events", 4,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
                ImGuiTableFlags_ScrollY,
            {0, std::max(60.f, ImGui::GetContentRegionAvail().y)})) {
      for (auto h : {"Frame", "Track", "Event", "Evidence"})
        ImGui::TableSetupColumn(h);
      ImGui::TableHeadersRow();
      ImGuiListClipper clip;
      std::vector<const J *> shown;
      for (auto &e : events) {
        if (selectedOnly && int(num(e, "track", -1)) != track)
          continue;
        if (*eventSearch && e.dump().find(eventSearch) == std::string::npos)
          continue;
        shown.push_back(&e);
      }
      clip.Begin(shown.size());
      while (clip.Step())
        for (int i = clip.DisplayStart; i < clip.DisplayEnd; i++) {
          auto &e = *shown[i];
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::PushID(i);
          if (ImGui::Selectable(str(e, "frame").c_str(),
                                frame == int(num(e, "frame")),
                                ImGuiSelectableFlags_SpanAllColumns)) {
            seek(int(num(e, "frame")));
            if (e.contains("track") && e["track"].is_number())
              pickTrack(e["track"].get<int>());
          }
          ImGui::PopID();
          ImGui::TableNextColumn();
          text(str(e, "track", "-"));
          ImGui::TableNextColumn();
          text(str(e, "kind"));
          ImGui::TableNextColumn();
          text(str(e, "detail"));
        }
      ImGui::EndTable();
    }
    ImGui::End();
  }
  void compare() {
    if (!ImGui::Begin("A/B comparison")) {
      ImGui::End();
      return;
    }
    auto &c = obj(bundle, "comparison");
    wrapped(str(c, "classification",
                str(c, "notice", "Select completed Before and After results")));
    text("Aligned frame: " + std::to_string(frame));
    auto &m = obj(c, "alignment");
    text("Timestamp ns: " + str(m, "timestamp_ns", "Not recorded"));
    if (c.contains("evidence_equal"))
      text("Recorded evidence equal: " + str(c, "evidence_equal"));
    wrapped("Numeric Track IDs are not physical identity across runs. Common "
            "unchanged fields are collapsed.");
    auto &ds = obj(c, "differences");
    for (auto it = ds.begin(); it != ds.end(); ++it)
      if (!arr(it.value(), "changed").empty())
        raw(("Track " + it.key()).c_str(), it.value());
    raw("Stage differences", arr(c, "stage_differences"));
    raw("Advanced comparison", c);
    wrapped("Jetson recording: Not Available");
    ImGui::End();
  }
  void start(const char *kind) {
    try {
      J override = J::object();
      for (auto &e : edits)
        if (!e.second.empty()) {
          J v = J::parse(e.second);
          for (auto &p : arr(opened, "parameters"))
            if (str(p, "name") == e.first && p.contains("current") &&
                v != p["current"])
              override[e.first] = v;
        }
      bool frozen = std::string(kind) == "what-if";
      if (frozen) {
        J only = J::object();
        if (override.contains("vehicle_min_score"))
          only["vehicle_min_score"] = override["vehicle_min_score"];
        override = only;
      }
      J r = {{"root", root},
             {"dataset", dataset},
             {"mode", kind},
             {"label", label},
             {"overrides", override},
             {"frame", frame},
             {"detail_start", rangeStart},
             {"detail_end", rangeEnd},
             {"end",
              std::string(kind) == "full" ? total : std::max(rangeEnd, frame)},
             {"run", runs[anchor]},
             {"baseline", runs[0]},
             {"track", track >= 0 ? J(track) : J(nullptr)}};
      if (frozen) {
        r["detail_start"] = frame;
        r["detail_end"] = frame;
        r["end"] = frame;
      }
      ask("start", {{"request", r}});
    } catch (std::exception &e) {
      error = std::string("Invalid override JSON: ") + e.what();
    }
  }
  void parameters() {
    if (!ImGui::Begin("Parameter Lab")) {
      ImGui::End();
      return;
    }
    wrapped("Isolated overrides only. Operating YAML and source datasets "
            "remain unchanged.");
    bool compatible = yes(obj(opened, "metadata"), "prefix_compatible");
    ImGui::TextColored(
        compatible ? ImVec4(.3, .65, .4, 1) : ImVec4(.8, .5, .25, 1),
        "Dataset preprocessing compatibility: %s",
        compatible ? "matching prefix"
                   : "different prefix / replay validation required");
    ImGui::InputText("Experiment", label, sizeof(label));
    if (ImGui::Button("Dataset Replay"))
      start("dataset");
    ImGui::SameLine();
    if (ImGui::Button("Full Replay"))
      start("full");
    ImGui::SameLine();
    if (ImGui::Button("Frozen What-if"))
      start("what-if");
    wrapped("Frozen What-if evaluates vehicle_min_score from a same-binary "
            "saved detection snapshot. It does not advance KF / Association. "
            "Full Replay processes the complete original bag.");
    ImGui::InputText("Find parameter", paramSearch, sizeof(paramSearch));
    if (ImGui::Button("Clear overrides"))
      edits.clear();
    if (ImGui::BeginTable(
            "parameters", 4,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
                ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX,
            {0, std::max(100.f, ImGui::GetContentRegionAvail().y)})) {
      ImGui::TableSetupColumn("Parameter", ImGuiTableColumnFlags_WidthFixed,
                              210);
      ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthFixed, 100);
      ImGui::TableSetupColumn("Override (JSON)",
                              ImGuiTableColumnFlags_WidthFixed, 150);
      ImGui::TableSetupColumn("Unit / availability",
                              ImGuiTableColumnFlags_WidthFixed, 180);
      ImGui::TableHeadersRow();
      for (auto &p : arr(opened, "parameters")) {
        std::string name = str(p, "name");
        if (*paramSearch && name.find(paramSearch) == std::string::npos)
          continue;
        ImGui::PushID(name.c_str());
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        text(name);
        if (ImGui::IsItemHovered()) {
          ImGui::BeginTooltip();
          wrapped(p.dump(2));
          ImGui::EndTooltip();
        }
        ImGui::TableNextColumn();
        text(str(p, "current"));
        ImGui::TableNextColumn();
        char buffer[512];
        snprintf(buffer, sizeof(buffer), "%s", edits[name].c_str());
        ImGui::BeginDisabled(!yes(p, "editable"));
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputTextWithHint("##override", "unchanged", buffer,
                                     sizeof(buffer)))
          edits[name] = buffer;
        ImGui::EndDisabled();
        ImGui::TableNextColumn();
        text(str(p, "unit") + (yes(p, "requires_regeneration")
                                   ? " / prefix locked"
                               : yes(p, "whatif") ? " / What-if"
                                                  : " / Replay"));
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    ImGui::End();
  }
  void experiments() {
    if (!ImGui::Begin("Experiments")) {
      ImGui::End();
      return;
    }
    if (ImGui::Button("Refresh history")) {
      ask("jobs");
      ask("open");
    }
    wrapped("Existing shared Lab history. Cancel affects only the selected Lab "
            "experiment, never external dataset generation.");
    for (auto it = jobs.rbegin(); it != jobs.rend(); ++it) {
      auto &j = *it;
      ImGui::PushID(str(j, "id").c_str());
      std::string title = str(j, "label") + " / " + str(j, "status") + "##job";
      if (ImGui::CollapsingHeader(title.c_str())) {
        text(str(j, "dataset") + " / " + str(j, "mode"));
        float progress = float(num(j, "progress_frame")) /
                         float(std::max(1., num(j, "progress_total", 1)));
        ImGui::ProgressBar(std::min(1.f, progress), {-1, 6}, "");
        text(str(j, "progress_frame") + " / " + str(j, "progress_total"));
        bool active =
            str(j, "status") == "queued" || str(j, "status") == "running";
        if (active) {
          if (ImGui::Button("Cancel this run"))
            ask("cancel", {{"job", str(j, "id")}});
        } else if (ImGui::Button("Restore experiment"))
          ask("restore", {{"job", str(j, "id")}});
        if (j.contains("error"))
          wrapped(str(j, "error"));
        raw("Result / provenance", j);
      }
      ImGui::PopID();
    }
    ImGui::End();
  }
  void toggleFocus() {
    fullFocus = !fullFocus;
    if (!fullFocus)
      layout = true;
  }
  void render() {
    header();
    if (ImGui::IsKeyPressed(ImGuiKey_F11))
      toggleFocus();
    if (ImGui::IsKeyPressed(ImGuiKey_Escape) && fullFocus)
      toggleFocus();
    if (!ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Space))
      playing = !playing;
    if (fullFocus) {
      bev(0);
      return;
    }
    dock();
    bev(0);
    if (mode == "Compare")
      bev(1);
    trackInspector();
    gates();
    geometry();
    fov();
    parameters();
    experiments();
    timeline();
    metrics();
    trajectory();
    eventPanel();
    compare();
    if (focusPreset) {
      focusPreset = false;
      selectPanel(mode == "Tune" ? "Parameter Lab" : "Track inspector");
      selectPanel("Decision / Gates");
      selectPanel(mode == "Tune" || mode == "Compare" ? "ImPlot metrics"
                                                      : "Timeline");
    }
  }
  J selection() const {
    return {
        {"schema", 1},
        {"application", "General Perception Studio"},
        {"root", root},
        {"dataset", dataset},
        {"dataset_path", str(obj(opened, "metadata"), "path")},
        {"manifest_sha256", str(obj(opened, "metadata"), "manifest_sha256")},
        {"point_index_space",
         "Prepared-frame compact indices; LOD point field 6"},
        {"frame", frame},
        {"timestamp_ns", obj(data(), "index").value("stamp_ns", J(nullptr))},
        {"runs", J::array({runs[0], runs[1]})},
        {"track", track},
        {"cluster", cluster},
        {"camera", {{"x", camera.x}, {"y", camera.y}, {"width", camera.width}}},
        {"anchor", anchor},
        {"layers", layers()},
        {"mode", mode},
        {"actual_points", num(data(), "actual_points")},
        {"lod_points", num(data(), "rendered_points")},
        {"cluster_count", arr(data(), "clusters").size()},
        {"shown_points", J::array({visiblePoints[0], visiblePoints[1]})},
        {"comparison", obj(bundle, "comparison")},
        {"notice", "Actual points distinct from dashed memory/prediction. "
                   "Display LOD only. No algorithm inputs modified."}};
  }
  void exportPNG(const fs::path &file) {
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    std::vector<unsigned char> pixels(size_t(w) * h * 3);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    fs::create_directories(file.parent_path());
    FILE *f = fopen(file.c_str(), "wb");
    if (!f)
      throw std::runtime_error("PNG file open failed");
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                              nullptr, nullptr);
    png_infop info = png_create_info_struct(png);
    if (setjmp(png_jmpbuf(png))) {
      png_destroy_write_struct(&png, &info);
      fclose(f);
      throw std::runtime_error("PNG export failed");
    }
    png_init_io(png, f);
    png_set_IHDR(png, info, w, h, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    std::vector<png_bytep> rows(h);
    for (int y = 0; y < h; y++)
      rows[y] = pixels.data() + size_t(h - y - 1) * w * 3;
    png_write_image(png, rows.data());
    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);
    fclose(f);
    fs::path meta = file;
    meta.replace_extension(".json");
    J evidence=selection();
    evidence["tracking_line_data"]={graphs[0],graphs[1]};
    evidence["tracking_frame_range"]={graphMin,graphMax};
    writeJson(meta,evidence);
    status = "Exported " + file.string();
  }
};

int main(int argc, char **argv) {
  signal(SIGPIPE, SIG_IGN);
  fs::path state = "/home/sjjin/construct_ws/general_debug_lab", screens;
  bool validate = false, validateBuffer = false, validateLines = false;
  std::vector<std::string> extraRoots;
  int width = 1920, height = 1080;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--state" && i + 1 < argc)
      state = argv[++i];
    else if (a == "--validate" && i + 1 < argc) {
      screens = argv[++i];
      validate = true;
    } else if (a == "--validate-lines" && i + 1 < argc) {
      screens=argv[++i];validateLines=true;
    } else if (a == "--dataset-root" && i + 1 < argc)
      extraRoots.push_back(argv[++i]);
    else if (a == "--validate-buffer" && i + 1 < argc) {
      screens = argv[++i];
      validateBuffer = true;
    } else if (a == "--width" && i + 1 < argc)
      width = std::stoi(argv[++i]);
    else if (a == "--height" && i + 1 < argc)
      height = std::stoi(argv[++i]);
    else if (a == "--help") {
      std::cout << "General Perception Studio [--state DIR] [--dataset-root "
                   "DIR] [--width N "
                   "--height N] [--validate SCREENSHOT_DIR]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << a << '\n';
      return 2;
    }
  }
  fs::create_directories(state);
  int lock = open((state / "studio.lock").c_str(), O_WRONLY | O_CREAT, 0644);
  if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB)) {
    std::cerr << "General Perception Studio is already running\n";
    return 2;
  }
  glfwSetErrorCallback([](int code, const char *msg) {
    std::cerr << "GLFW " << code << ": " << msg << '\n';
  });
  if (!glfwInit())
    return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow *window = glfwCreateWindow(
      width, height, "General Perception Studio", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  auto &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  std::string ini = (state / "studio_layout.ini").string();
  io.IniFilename = ini.c_str();
  io.Fonts->AddFontFromFileTTF(
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16);
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");
  int exitCode = 0;
  try {
    Studio app(window, state, extraRoots);
    if (fs::exists(ini) && !validate)
      app.layout = false;
    if (validate) {
      app.root = "local";
      app.dataset = "20261023";
      app.frame = 80;
      app.track = 6;
      app.dark = false;
      app.setLayers({{"points", true},
                     {"foreground", true},
                     {"roi_nonforeground", true},
                     {"outside_roi", true}});
      app.focusOnly = false;
      app.mode = "Explore";
      app.layout = true;
      app.theme();
    }
    if (validateBuffer) {
      app.root = "z";
      app.dataset = "rosbag2_2026_10_07-07_40_05";
      app.frame = 80;
      app.track = -1;
      app.layout = true;
    }
    int phase = 0, settle = 0;
    double began = now();
    J result = J::object();
    while (!glfwWindowShouldClose(window) && !app.quit) {
      glfwPollEvents();
      app.process();
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      app.render();
      ImGui::Render();
      int w, h;
      glfwGetFramebufferSize(window, &w, &h);
      glViewport(0, 0, w, h);
      glClearColor(.12, .13, .12, 1);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      if (!app.exportRequest.empty()) {
        auto ticks =
            std::chrono::system_clock::now().time_since_epoch().count();
        app.exportPNG(state / "exports" / ("studio_" + std::to_string(ticks)) /
                      (app.exportRequest + ".png"));
        app.exportRequest.clear();
      }
      if(validateLines) {
        if(now()-began>150){result["error"]=app.error.empty()?"Tracking line validation timeout":app.error;
          result["phase"]=phase;result["frame"]=app.frame;result["loaded"]=app.loaded();result["status"]=app.status;result["track"]=app.track;result["root"]=app.root;result["dataset"]=app.dataset;result["graphs"]=J::array({arr(app.graphs[0],"samples").size(),arr(app.graphs[1],"samples").size()});result["runs"]=J::array({app.runLabels[0],app.runLabels[1]});app.exportPNG(screens/"timeout.png");writeJson(screens/"tracking_line_validation.json",result);exitCode=2;break;}
        if(phase==0&&app.loaded()&&!arr(app.graphs[0],"samples").empty()&&!arr(app.graphs[1],"samples").empty()) {
          app.validateTrackingLines=true;app.validationTargetFrame=app.frame+1;
          app.mode="Compare";app.layout=true;app.graphMin=app.frame-49;app.graphMax=app.frame+61;
          app.trajectoryFit=true;phase=1;settle=0;
        }
        if(phase==1) {
          selectPanel("Timeline");
          if(++settle>12&&app.validationTimelinePoint.x>0) {
            result["timeline_target"]=app.validationTargetFrame;
            app.exportPNG(screens/"01_tracking_lines.png");
            auto p=app.validationTimelinePoint;glfwSetCursorPos(window,p.x,p.y);ImGui::GetIO().AddMousePosEvent(p.x,p.y);
            phase=20;settle=0;
          }
        } else if(phase==20&&++settle>4) {ImGui::GetIO().AddMouseButtonEvent(0,true);phase=2;settle=0;}
        else if(phase==2&&++settle>2) {ImGui::GetIO().AddMousePosEvent(app.validationTimelinePoint.x,app.validationTimelinePoint.y);ImGui::GetIO().AddMouseButtonEvent(0,false);phase=3;}
        else if(phase==3&&app.loaded()&&app.frame==app.validationTargetFrame&&app.anchor==1&&!app.fitGraphPending) {
          const auto * tr=app.selected();
          result["timeline_mouse_click"]={{"expected_frame",app.validationTargetFrame},
            {"actual_frame",num(app.data(),"frame")},{"track",app.track},{"side",app.anchor},
            {"selected_id",tr?num(*tr,"id"):-1},{"gates_available",tr&&!arr(*tr,"gate_inspector").empty()},
            {"fit_width_m",app.camera.width}};
          app.validationTargetFrame=app.frame+1;app.trajectoryFit=true;phase=4;settle=0;
          selectPanel("World XY history");
        } else if(phase==4) {
          selectPanel("World XY history");
          if(++settle>12&&app.validationTrajectoryPoint.x>0) {
            app.exportPNG(screens/"02_world_xy_lines.png");
            auto p=app.validationTrajectoryPoint;glfwSetCursorPos(window,p.x,p.y);ImGui::GetIO().AddMousePosEvent(p.x,p.y);
            phase=21;settle=0;
          }
        } else if(phase==21&&++settle>4) {ImGui::GetIO().AddMouseButtonEvent(0,true);phase=5;settle=0;}
        else if(phase==5&&++settle>2) {ImGui::GetIO().AddMousePosEvent(app.validationTrajectoryPoint.x,app.validationTrajectoryPoint.y);ImGui::GetIO().AddMouseButtonEvent(0,false);phase=6;}
        else if(phase==6&&app.loaded()&&app.frame==app.validationTargetFrame&&app.anchor==1&&!app.fitGraphPending) {
          const auto * tr=app.selected();
          result["trajectory_mouse_click"]={{"expected_frame",app.validationTargetFrame},
            {"actual_frame",num(app.data(),"frame")},{"track",app.track},{"side",app.anchor},
            {"selected_id",tr?num(*tr,"id"):-1},{"gates_available",tr&&!arr(*tr,"gate_inspector").empty()},
            {"fit_width_m",app.camera.width}};
          selectPanel("Decision / Gates");phase=7;settle=0;
        } else if(phase==7&&++settle>12) {
          app.exportPNG(screens/"03_clicked_frame_gates.png");
          selectPanel("ImPlot metrics");phase=8;settle=0;
        } else if(phase==8&&++settle>12) {
          app.exportPNG(screens/"04_angular_lines.png");
          result["input_method"]="ImGui actual mouse down/up events on rendered plot coordinates";
          result["passed"]=app.frame==app.validationTargetFrame&&app.track==11&&app.anchor==1&&app.camera.width<20;
          writeJson(screens/"tracking_line_validation.json",result);
          if(!result["passed"].get<bool>())exitCode=2;
          break;
        }
      }
      if (validateBuffer) {
        if (now() - began > 90) {
          exitCode = 2;
          break;
        }
        if (app.loaded() && phase == 0) {
          app.fitVisible();
          app.rangeStart = 80;
          app.rangeEnd = 82;
          app.ask("buffer-config", {{"auto", false}, {"limit_mb", 64}});
          app.ask("buffer-start", {{"start", 80}, {"end", 82}});
          phase = 1;
        }
        if (phase == 1 && str(app.buffer, "status") == "complete") {
          app.showBuffer = true;
          phase = 2;
        }
        if (phase == 2 && ++settle > 12) {
          app.exportPNG(screens / "buffer.png");
          writeJson(screens / "buffer_status.json", app.buffer);
          break;
        }
      }
      if (validate) {
        if (now() - began > 150) {
          result["error"] =
              app.error.empty() ? "Validation timeout" : app.error;
          writeJson(screens / "validation.json", result);
          exitCode = 2;
          break;
        }
        if (app.loaded() && app.requests.empty()) {
          settle++;
          if (settle > 5) {
            settle = 0;
            if (phase == 0) {
              app.fitVisible();
              phase++;
            } else if (phase == 1) {
              app.exportPNG(screens / "01_explore.png");
              result["frame80"] = app.selection();
              double before = app.camera.width;
              app.focusTrack(true);
              result["focus_preserves_scale"] = app.camera.width == before;
              app.follow = false;
              if (const J *t = app.selected())
                result["track6"] = *t;
              app.mode = "Compare";
              app.layout = true;
              phase++;
            } else if (phase == 2) {
              app.exportPNG(screens / "02_compare.png");
              app.mode = "Tune";
              app.layout = true;
              phase++;
            } else if (phase == 3) {
              app.exportPNG(screens / "03_tune.png");
              app.mode = "Explore";
              app.layout = true;
              app.fitSelected();
              app.focusOnly = true;
              app.needsUpload = true;
              phase++;
            } else if (phase == 4) {
              app.exportPNG(screens / "04_focus_selected.png");
              app.focusOnly = false;
              app.foreground = true;
              app.roiPoints = false;
              app.outside = false;
              app.needsUpload = true;
              phase++;
            } else if (phase == 5) {
              app.fitVisible();
              app.exportPNG(screens / "05_foreground.png");
              app.dark = true;
              app.theme();
              phase++;
            } else if (phase == 6) {
              app.exportPNG(screens / "06_dark.png");
              glfwSetWindowSize(window, 1100, 760);
              phase++;
            } else if (phase == 7) {
              app.exportPNG(screens / "07_small.png");
              glfwSetWindowSize(window, 1920, 1080);
              app.fullFocus = true;
              phase++;
            } else if (phase == 8) {
              app.exportPNG(screens / "08_focus_view.png");
              app.fullFocus = false;
              app.mode = "Explore";
              app.layout = true;
              phase = 11;
            } else if (phase == 11) {
              selectPanel("FOV / Depth");
              phase++;
            } else if (phase == 12) {
              app.exportPNG(screens / "10_fov_depth.png");
              selectPanel("Track inspector");
              selectPanel("Decision / Gates");
              phase++;
            } else if (phase == 13) {
              app.exportPNG(screens / "11_track_gate.png");
              selectPanel("Experiments");
              phase++;
            } else if (phase == 14) {
              app.exportPNG(screens / "12_history.png");
              for (auto &j : app.jobs)
                if (str(j, "label") == "native_ui_frozen_validation" &&
                    str(j, "status") == "completed") {
                  app.ask("restore", {{"job", str(j, "id")}});
                  break;
                }
              phase = 15;
            } else if (phase == 15) {
              selectPanel("Decision / Gates");
              selectPanel("Parameter Lab");
              snprintf(app.paramSearch, sizeof(app.paramSearch),
                       "vehicle_min_score");
              phase++;
            } else if (phase == 16) {
              app.exportPNG(screens / "13_restored_whatif.png");
              result["restored_whatif"] = app.whatif;
              selectPanel("Decision / Gates");
              phase = 17;
            } else if (phase == 17) {
              app.exportPNG(screens / "14_restored_gate.png");
              app.dataset = "20261020";
              app.root = "local";
              app.frame = 2500;
              app.open();
              phase = 9;
            } else if (phase == 9) {
              app.fitVisible();
              phase++;
            } else if (phase == 10) {
              app.exportPNG(screens / "09_long_bag.png");
              result["long_bag"] = app.selection();
              result["worker_processes"] = 1;
              result["server"] = false;
              result["qt"] = false;
              writeJson(screens / "validation.json", result);
              break;
            }
          }
        }
      }
      glfwSwapBuffers(window);
      if (!app.playing && io.MouseDelta.x == 0 && io.MouseDelta.y == 0 &&
          !ImGui::IsAnyItemActive())
        glfwWaitEventsTimeout(.02);
      if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
        glfwWaitEventsTimeout(.1);
    }
  } catch (std::exception &e) {
    std::cerr << "STUDIO ERROR: " << e.what() << '\n';
    exitCode = 1;
  }
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  close(lock);
  return exitCode;
}
