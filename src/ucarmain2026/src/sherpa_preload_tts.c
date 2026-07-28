// sherpa-sentence-clips-v7.1-20260728
//
// 小车启动时创建一个 Sherpa-ONNX OfflineTts，模型加载完成后等待
// 唯一一次 0～2 段物品名任务。生成完成后先销毁模型，再回复客户端并退出。

#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <locale.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define REQUEST_MAGIC 0x54545331U
#define RESPONSE_MAGIC 0x54545352U
#define MAX_TASK_COUNT 2U
#define MAX_FIELD_BYTES (1024U * 1024U)
#define SERVER_BUILD_ID "sherpa-sentence-clips-v7.1-20260728"
#define SERVER_PROTOCOL_ID "item-cache-0-to-2-v1"

// 以下声明与 sherpa-onnx v1.10.45 的 c-api.h 保持一致。
typedef struct SherpaOnnxOfflineTtsVitsModelConfig {
  const char *model;
  const char *lexicon;
  const char *tokens;
  const char *data_dir;
  float noise_scale;
  float noise_scale_w;
  float length_scale;
  const char *dict_dir;
} SherpaOnnxOfflineTtsVitsModelConfig;

typedef struct SherpaOnnxOfflineTtsMatchaModelConfig {
  const char *acoustic_model;
  const char *vocoder;
  const char *lexicon;
  const char *tokens;
  const char *data_dir;
  float noise_scale;
  float length_scale;
  const char *dict_dir;
} SherpaOnnxOfflineTtsMatchaModelConfig;

typedef struct SherpaOnnxOfflineTtsKokoroModelConfig {
  const char *model;
  const char *voices;
  const char *tokens;
  const char *data_dir;
  float length_scale;
  const char *dict_dir;
  const char *lexicon;
} SherpaOnnxOfflineTtsKokoroModelConfig;

typedef struct SherpaOnnxOfflineTtsModelConfig {
  SherpaOnnxOfflineTtsVitsModelConfig vits;
  int32_t num_threads;
  int32_t debug;
  const char *provider;
  SherpaOnnxOfflineTtsMatchaModelConfig matcha;
  SherpaOnnxOfflineTtsKokoroModelConfig kokoro;
} SherpaOnnxOfflineTtsModelConfig;

typedef struct SherpaOnnxOfflineTtsConfig {
  SherpaOnnxOfflineTtsModelConfig model;
  const char *rule_fsts;
  int32_t max_num_sentences;
  const char *rule_fars;
  float silence_scale;
} SherpaOnnxOfflineTtsConfig;

typedef struct SherpaOnnxGeneratedAudio {
  const float *samples;
  int32_t n;
  int32_t sample_rate;
} SherpaOnnxGeneratedAudio;

typedef struct SherpaOnnxOfflineTts SherpaOnnxOfflineTts;

extern const SherpaOnnxOfflineTts *SherpaOnnxCreateOfflineTts(
    const SherpaOnnxOfflineTtsConfig *config);
extern void SherpaOnnxDestroyOfflineTts(
    const SherpaOnnxOfflineTts *tts);
extern const SherpaOnnxGeneratedAudio *SherpaOnnxOfflineTtsGenerate(
    const SherpaOnnxOfflineTts *tts, const char *text, int32_t sid,
    float speed);
extern void SherpaOnnxDestroyOfflineTtsGeneratedAudio(
    const SherpaOnnxGeneratedAudio *audio);
extern int32_t SherpaOnnxWriteWave(
    const float *samples, int32_t n, int32_t sample_rate,
    const char *filename);

typedef struct TtsTask {
  char *text;
  char *output_file;
} TtsTask;

static char g_socket_path[sizeof(((struct sockaddr_un *)0)->sun_path)];
static int g_listen_fd = -1;
static int g_client_fd = -1;

static double monotonic_seconds(void) {
  struct timespec value;
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    return 0.0;
  }
  return (double)value.tv_sec + (double)value.tv_nsec / 1000000000.0;
}

static int parse_thread_count(void) {
  const char *text = getenv("TTS_NUM_THREADS");
  char *end = NULL;
  long value = 4;

  if (text != NULL && text[0] != '\0') {
    errno = 0;
    value = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
      value = 4;
    }
  }

  if (value < 1) {
    value = 1;
  } else if (value > 8) {
    value = 8;
  }
  return (int)value;
}

static int build_path(
    char *output, size_t output_size,
    const char *package_dir, const char *relative_path) {
  const int written = snprintf(
      output, output_size, "%s/%s", package_dir, relative_path);
  if (written < 0 || (size_t)written >= output_size) {
    fprintf(stderr, "[TTS] 路径过长：%s/%s\n", package_dir, relative_path);
    return 0;
  }
  return 1;
}

static int require_readable_file(const char *description, const char *path) {
  if (access(path, R_OK) != 0) {
    fprintf(
        stderr, "[TTS] 缺少%s或文件不可读：%s\n",
        description, path);
    return 0;
  }
  return 1;
}

static void close_descriptors_and_unlink(void) {
  if (g_client_fd >= 0) {
    close(g_client_fd);
    g_client_fd = -1;
  }
  if (g_listen_fd >= 0) {
    close(g_listen_fd);
    g_listen_fd = -1;
  }
  if (g_socket_path[0] != '\0') {
    unlink(g_socket_path);
  }
}

static void handle_termination_signal(int signal_number) {
  close_descriptors_and_unlink();
  _exit(128 + signal_number);
}

static int install_signal_handlers(void) {
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_handler = handle_termination_signal;
  sigemptyset(&action.sa_mask);

  if (sigaction(SIGINT, &action, NULL) != 0 ||
      sigaction(SIGTERM, &action, NULL) != 0) {
    fprintf(stderr, "[TTS] 无法安装退出信号处理器：%s\n", strerror(errno));
    return 0;
  }

  signal(SIGPIPE, SIG_IGN);
  return 1;
}

static int read_exact(int fd, void *buffer, size_t byte_count) {
  unsigned char *cursor = (unsigned char *)buffer;
  size_t received = 0;

  while (received < byte_count) {
    const ssize_t result = read(fd, cursor + received, byte_count - received);
    if (result == 0) {
      return 0;
    }
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return 0;
    }
    received += (size_t)result;
  }
  return 1;
}

static int write_exact(int fd, const void *buffer, size_t byte_count) {
  const unsigned char *cursor = (const unsigned char *)buffer;
  size_t written = 0;

  while (written < byte_count) {
    const ssize_t result = write(fd, cursor + written, byte_count - written);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return 0;
    }
    if (result == 0) {
      return 0;
    }
    written += (size_t)result;
  }
  return 1;
}

static int read_u32(int fd, uint32_t *value) {
  uint32_t network_value = 0;
  if (!read_exact(fd, &network_value, sizeof(network_value))) {
    return 0;
  }
  *value = ntohl(network_value);
  return 1;
}

static char *read_string(int fd, const char *description) {
  uint32_t length = 0;
  if (!read_u32(fd, &length)) {
    fprintf(stderr, "[TTS] 读取%s长度失败\n", description);
    return NULL;
  }
  if (length == 0 || length > MAX_FIELD_BYTES) {
    fprintf(
        stderr, "[TTS] %s长度非法：%u\n",
        description, (unsigned int)length);
    return NULL;
  }

  char *value = (char *)calloc((size_t)length + 1U, 1U);
  if (value == NULL) {
    fprintf(stderr, "[TTS] 为%s分配内存失败\n", description);
    return NULL;
  }
  if (!read_exact(fd, value, length)) {
    fprintf(stderr, "[TTS] 读取%s内容失败\n", description);
    free(value);
    return NULL;
  }
  if (memchr(value, '\0', length) != NULL) {
    fprintf(stderr, "[TTS] %s含有非法空字符\n", description);
    free(value);
    return NULL;
  }
  return value;
}

static void free_tasks(TtsTask *tasks, uint32_t task_count) {
  if (tasks == NULL) {
    return;
  }
  for (uint32_t index = 0; index < task_count; ++index) {
    free(tasks[index].text);
    free(tasks[index].output_file);
  }
  free(tasks);
}

static TtsTask *read_request(int fd, uint32_t *task_count) {
  uint32_t magic = 0;
  uint32_t count = 0;

  if (!read_u32(fd, &magic) || !read_u32(fd, &count)) {
    fprintf(stderr, "[TTS] 无法读取客户端请求头\n");
    return NULL;
  }
  if (magic != REQUEST_MAGIC) {
    fprintf(stderr, "[TTS] 客户端协议标识错误\n");
    return NULL;
  }
  if (count > MAX_TASK_COUNT) {
    fprintf(
        stderr, "[TTS] 本进程一次最多接受两个物品名，实际收到 %u 个\n",
        (unsigned int)count);
    return NULL;
  }

  TtsTask *tasks = (TtsTask *)calloc(
      count == 0U ? 1U : count, sizeof(TtsTask));
  if (tasks == NULL) {
    fprintf(stderr, "[TTS] 无法为任务分配内存\n");
    return NULL;
  }

  for (uint32_t index = 0; index < count; ++index) {
    char description[64];
    snprintf(
        description, sizeof(description),
        "第 %u 段文本", (unsigned int)(index + 1U));
    tasks[index].text = read_string(fd, description);
    if (tasks[index].text == NULL) {
      free_tasks(tasks, count);
      return NULL;
    }

    snprintf(
        description, sizeof(description),
        "第 %u 段输出路径", (unsigned int)(index + 1U));
    tasks[index].output_file = read_string(fd, description);
    if (tasks[index].output_file == NULL) {
      free_tasks(tasks, count);
      return NULL;
    }
  }

  *task_count = count;
  return tasks;
}

static int send_response(int fd, uint32_t status, const char *message) {
  const size_t message_length = strlen(message);
  if (message_length > MAX_FIELD_BYTES) {
    return 0;
  }

  uint32_t header[3];
  header[0] = htonl(RESPONSE_MAGIC);
  header[1] = htonl(status);
  header[2] = htonl((uint32_t)message_length);

  return write_exact(fd, header, sizeof(header)) &&
      write_exact(fd, message, message_length);
}

static int create_listening_socket(const char *socket_path) {
  if (strlen(socket_path) >= sizeof(g_socket_path)) {
    fprintf(stderr, "[TTS] Unix Socket 路径过长：%s\n", socket_path);
    return -1;
  }

  snprintf(g_socket_path, sizeof(g_socket_path), "%s", socket_path);
  unlink(g_socket_path);

  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    fprintf(stderr, "[TTS] 创建 Unix Socket 失败：%s\n", strerror(errno));
    return -1;
  }

  struct sockaddr_un address;
  memset(&address, 0, sizeof(address));
  address.sun_family = AF_UNIX;
  snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path);

  if (bind(fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
    fprintf(
        stderr, "[TTS] 绑定 Unix Socket 失败：%s\n",
        strerror(errno));
    close(fd);
    return -1;
  }
  if (chmod(socket_path, 0660) != 0) {
    fprintf(stderr, "[TTS] 设置 Socket 权限失败：%s\n", strerror(errno));
    close(fd);
    unlink(socket_path);
    return -1;
  }
  if (listen(fd, 1) != 0) {
    fprintf(stderr, "[TTS] 监听 Unix Socket 失败：%s\n", strerror(errno));
    close(fd);
    unlink(socket_path);
    return -1;
  }

  return fd;
}

static int synthesize_tasks(
    const SherpaOnnxOfflineTts *tts,
    const TtsTask *tasks, uint32_t task_count) {
  for (uint32_t index = 0; index < task_count; ++index) {
    fprintf(
        stderr, "[TTS] 正在生成第 %u/%u 段：%s\n",
        (unsigned int)(index + 1U), (unsigned int)task_count,
        tasks[index].output_file);
    fflush(stderr);

    const double generate_start = monotonic_seconds();
    const SherpaOnnxGeneratedAudio *audio =
        SherpaOnnxOfflineTtsGenerate(tts, tasks[index].text, 0, 1.0f);
    const double generate_end = monotonic_seconds();

    if (audio == NULL || audio->samples == NULL ||
        audio->n <= 0 || audio->sample_rate <= 0) {
      fprintf(
          stderr, "[TTS] 第 %u 段推理没有返回有效音频\n",
          (unsigned int)(index + 1U));
      if (audio != NULL) {
        SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
      }
      return 0;
    }

    const int write_succeeded = SherpaOnnxWriteWave(
        audio->samples, audio->n, audio->sample_rate,
        tasks[index].output_file);
    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);

    if (!write_succeeded) {
      fprintf(
          stderr, "[TTS] 第 %u 段 WAV 写入失败：%s\n",
          (unsigned int)(index + 1U), tasks[index].output_file);
      return 0;
    }

    fprintf(
        stderr, "[TTS] 第 %u 段完成，推理耗时 %.3f 秒\n",
        (unsigned int)(index + 1U), generate_end - generate_start);
  }
  return 1;
}

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");

  if (argc == 2 && strcmp(argv[1], "--version") == 0) {
    printf("%s %s\n", SERVER_BUILD_ID, SERVER_PROTOCOL_ID);
    return 0;
  }

  if (argc != 3) {
    fprintf(stderr, "用法：%s <工程目录> <Unix Socket 路径>\n", argv[0]);
    return 2;
  }

  const char *package_dir = argv[1];
  const char *socket_path = argv[2];
  const int num_threads = parse_thread_count();

  char acoustic_model[PATH_MAX];
  char vocoder[PATH_MAX];
  char lexicon[PATH_MAX];
  char tokens[PATH_MAX];
  char dict_dir[PATH_MAX];
  char phone_fst[PATH_MAX];
  char date_fst[PATH_MAX];
  char number_fst[PATH_MAX];
  char rule_fsts[PATH_MAX * 3 + 3];

  if (!build_path(
          acoustic_model, sizeof(acoustic_model), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/model-steps-3.onnx") ||
      !build_path(
          vocoder, sizeof(vocoder), package_dir,
          "third_party/sherpa_onnx/models/hifigan_v1.onnx") ||
      !build_path(
          lexicon, sizeof(lexicon), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/lexicon.txt") ||
      !build_path(
          tokens, sizeof(tokens), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/tokens.txt") ||
      !build_path(
          dict_dir, sizeof(dict_dir), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/dict") ||
      !build_path(
          phone_fst, sizeof(phone_fst), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/phone.fst") ||
      !build_path(
          date_fst, sizeof(date_fst), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/date.fst") ||
      !build_path(
          number_fst, sizeof(number_fst), package_dir,
          "third_party/sherpa_onnx/models/"
          "matcha-icefall-zh-baker/number.fst")) {
    return 1;
  }

  const int rule_written = snprintf(
      rule_fsts, sizeof(rule_fsts), "%s,%s,%s",
      phone_fst, date_fst, number_fst);
  if (rule_written < 0 || (size_t)rule_written >= sizeof(rule_fsts)) {
    fprintf(stderr, "[TTS] 规则 FST 路径过长\n");
    return 1;
  }

  if (!require_readable_file("Matcha 声学模型", acoustic_model) ||
      !require_readable_file("HiFi-GAN 声码器", vocoder) ||
      !require_readable_file("Matcha 词典", lexicon) ||
      !require_readable_file("Matcha tokens", tokens) ||
      !require_readable_file("phone.fst", phone_fst) ||
      !require_readable_file("date.fst", date_fst) ||
      !require_readable_file("number.fst", number_fst) ||
      !install_signal_handlers()) {
    return 1;
  }

  g_listen_fd = create_listening_socket(socket_path);
  if (g_listen_fd < 0) {
    return 1;
  }

  SherpaOnnxOfflineTtsConfig config;
  memset(&config, 0, sizeof(config));
  config.model.matcha.acoustic_model = acoustic_model;
  config.model.matcha.vocoder = vocoder;
  config.model.matcha.lexicon = lexicon;
  config.model.matcha.tokens = tokens;
  config.model.matcha.dict_dir = dict_dir;
  config.model.matcha.noise_scale = 0.667f;
  config.model.matcha.length_scale = 1.0f;
  config.model.num_threads = num_threads;
  config.model.debug = 0;
  config.model.provider = "cpu";
  config.rule_fsts = rule_fsts;
  config.max_num_sentences = 1;
  config.silence_scale = 0.2f;

  fprintf(
      stderr,
      "[TTS] 开机预加载进程已启动，版本=%s，协议=%s，"
      "线程数=%d，Socket=%s\n",
      SERVER_BUILD_ID, SERVER_PROTOCOL_ID, num_threads, socket_path);
  fprintf(stderr, "[TTS] 正在加载 Matcha 和 HiFi-GAN...\n");
  fflush(stderr);

  const double load_start = monotonic_seconds();
  const SherpaOnnxOfflineTts *tts = SherpaOnnxCreateOfflineTts(&config);
  const double load_end = monotonic_seconds();
  if (tts == NULL) {
    fprintf(stderr, "[TTS] 创建 OfflineTts 实例失败\n");
    close_descriptors_and_unlink();
    return 1;
  }

  fprintf(
      stderr, "[TTS] 模型预加载完成，耗时 %.3f 秒\n",
      load_end - load_start);
  fprintf(stderr, "[TTS] 等待唯一一次 0～2 个物品名合成任务...\n");
  fflush(stderr);

  do {
    g_client_fd = accept(g_listen_fd, NULL, NULL);
  } while (g_client_fd < 0 && errno == EINTR);

  uint32_t task_count = 0;
  TtsTask *tasks = NULL;
  int synthesis_succeeded = 0;

  if (g_client_fd < 0) {
    fprintf(stderr, "[TTS] 接收客户端连接失败：%s\n", strerror(errno));
  } else {
    struct timeval timeout;
    timeout.tv_sec = 30;
    timeout.tv_usec = 0;
    setsockopt(
        g_client_fd, SOL_SOCKET, SO_RCVTIMEO,
        &timeout, sizeof(timeout));

    tasks = read_request(g_client_fd, &task_count);
    if (tasks != NULL) {
      if (task_count == 0U) {
        fprintf(
            stderr,
            "[TTS] 两个物品音频均命中，跳过推理并立即释放模型\n");
        synthesis_succeeded = 1;
      } else {
        fprintf(
            stderr,
            "[TTS] 已收到 %u 个未命中物品名，开始合成；"
            "不会重新加载模型\n",
            (unsigned int)task_count);
        synthesis_succeeded = synthesize_tasks(tts, tasks, task_count);
      }
    }
  }

  free_tasks(tasks, task_count);

  // 必须先释放模型，再回复客户端。generate_task_audios.py 返回时，
  // NanoDet/OCR 已经不会与 TTS 模型同时占用内存。
  fprintf(stderr, "[TTS] 正在销毁 OfflineTts，立即释放模型内存...\n");
  SherpaOnnxDestroyOfflineTts(tts);
  fprintf(stderr, "[TTS] 模型已释放\n");

  if (g_client_fd >= 0) {
    char message[256];
    if (synthesis_succeeded) {
      snprintf(
          message, sizeof(message),
          "本次实际合成 %u 个物品名，TTS 模型已释放，"
          "预加载进程即将退出",
          (unsigned int)task_count);
    } else {
      snprintf(
          message, sizeof(message),
          "物品名 WAV 合成失败，TTS 模型已释放；请查看服务端日志");
    }
    if (!send_response(
            g_client_fd, synthesis_succeeded ? 0U : 1U, message)) {
      fprintf(stderr, "[TTS] 向客户端返回最终状态失败\n");
    }
  }

  close_descriptors_and_unlink();
  fprintf(stderr, "[TTS] 本进程正常退出\n");
  return synthesis_succeeded ? 0 : 1;
}
