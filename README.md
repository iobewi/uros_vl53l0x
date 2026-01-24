# micro-ROS + ToF (VL53L0X/mock)

Application micro-ROS pour ESP32-S3 publiant des messages `sensor_msgs/LaserScan` à partir de capteurs Time-of-Flight (ToF VL53L0X ou mock).

**Architecture modulaire** : core micro-ROS réutilisable + applications pluggables.

## ✨ Caractéristiques

- 🔌 **Architecture modulaire** : séparation core ROS / logique applicative
- 📡 **LaserScan ROS 2** : publication à ~10 Hz (configurable)
- 🎯 **Multi-capteurs ToF** : jusqu'à 84 capteurs VL53L0X en parallèle
- 🔄 **Reconnexion automatique** : supervision de l'agent micro-ROS
- 🎨 **LED de status** : indication visuelle (attente/connecté/erreur)
- ⚡ **QoS configurable** : BEST_EFFORT ou RELIABLE
- 🧪 **Mode mock** : tests sans hardware

## 🏗️ Architecture

### Vue d'ensemble

```
main.c
  └─→ uros_app_scan_start()
        ├─→ uros_core_create(config, app_interface)
        └─→ uros_core_start(core_ctx)
              ├─→ Main task: RCL init, executor spin, agent supervision
              └─→ Pub task: app_step() → rcl_publish()
```

### Modules

#### Core micro-ROS (`uros_core.c/h`)
**Responsabilités** : Infrastructure ROS générique et réutilisable
- Connexion agent micro-ROS (ping + reconnexion automatique)
- Lifecycle RCL (allocator, support, node, publisher, executor, timer)
- 2 tâches FreeRTOS : main (executor) + pub (publication)
- Gestion erreurs, backoff, métriques
- Thread-safety (mutex RCL)

**API publique** :
```c
uros_core_context_t *uros_core_create(const uros_core_config_t *config,
                                      const uros_app_interface_t *app);
bool uros_core_start(uros_core_context_t *ctx);
void uros_core_destroy(uros_core_context_t *ctx);
```

#### Application LaserScan (`uros_app_scan.c/h`)
**Responsabilités** : Logique métier spécifique au scan ToF
- Configuration scan (angles, ranges, timing)
- Orchestration `scan_engine` + `scan_builder`
- Callbacks : `scan_app_init()`, `scan_app_step()`, `scan_app_fini()`

**API publique** :
```c
bool uros_app_scan_start(void);  // Point d'entrée
```

#### Scan Engine (`scan_engine.c/h`)
**Responsabilités** : Orchestration des acquisitions ToF
- Acquisition snapshot ToF via `tof_provider`
- Timestamping (rmw_uros_epoch_nanos)
- Appel `scan_builder_fill()` pour construire le message

#### Scan Builder (`scan_builder.c/h`)
**Responsabilités** : Construction du message ROS LaserScan
- Init/deinit message avec buffers statiques ou malloc
- Mapping bins fixes (1 capteur → 1 bin)
- Gestion NaN pour bins non observés

#### ToF Provider (`tof_provider*.c/h`)
**Responsabilités** : Abstraction hardware
- `tof_provider_vl53.c` : Implémentation VL53L0X (I2C, IRQ)
- `tof_provider_mock.c` : Générateur de données simulées
- API commune : `tof_snapshot_read()`

### Interface application

Les applications s'intègrent au core via `uros_app_interface_t` :

```c
typedef struct {
    const rosidl_message_type_support_t *type_support;  // Type ROS
    size_t message_size;                                // Taille message
    bool (*app_init)(void *app_context);                // Init ressources
    bool (*app_step)(void *app_context, void *msg);     // Remplir message
    void (*app_fini)(void *app_context);                // Cleanup
    void *app_context;                                  // Contexte app
    size_t app_context_size;
} uros_app_interface_t;
```

**Allocation message** : Par le core (via `message_size`)
**Configuration** : Via struct `uros_core_config_t` (pas de dépendance Kconfig dans le core)

## 🚀 Quick Start

### Prérequis

- ESP-IDF v5.3+ ([installation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- Docker (pour l'agent micro-ROS)
- ROS 2 Humble (optionnel, pour tests)

### Build & Flash

```bash
# Configuration
idf.py menuconfig
# → Component config → micro-ROS → Configurer node, topic, QoS, etc.

# Build
idf.py build

# Flash
idf.py flash monitor
```

### Lancer l'agent micro-ROS

```bash
# USB-CDC (ESP32-S3)
docker run -it --rm --net=host \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0

# Serial classique
docker run -it --rm --net=host \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 -b 115200
```

### Vérifier la publication

```bash
# Fréquence de publication
ros2 topic hz /tof_scan

# Afficher un message
ros2 topic echo /tof_scan --once

# Visualiser dans RViz2
ros2 run rviz2 rviz2
# Ajouter LaserScan display avec topic /tof_scan
```

## 🔧 Configuration

### Kconfig principal (`main/Kconfig.projbuild`)

**Micro-ROS** :
- `CONFIG_MICRO_ROS_NODE_NAME` : Nom du nœud ROS (défaut: `tof_node`)
- `CONFIG_MICRO_ROS_TOPIC_NAME` : Topic de publication (défaut: `/tof_scan`)
- `CONFIG_MICRO_ROS_DOMAIN_ID` : Domain ID ROS 2 (défaut: `0`)
- `CONFIG_MICRO_ROS_TIMER_PERIOD_MS` : Période de publication (défaut: `100` ms)

**QoS** :
- `CONFIG_MICRO_ROS_QOS_RELIABLE` : Reliable (vs BEST_EFFORT)
- `CONFIG_MICRO_ROS_QOS_DEPTH` : Profondeur de la queue (défaut: `10`)

**Scan** :
- `CONFIG_MICRO_ROS_SCAN_BINS` : Nombre de bins (défaut: `84`)
- `CONFIG_MICRO_ROS_SCAN_FRAME_ID` : Frame ID (défaut: `base_link`)

**ToF Provider** :
- `CONFIG_TOF_PROVIDER_MOCK` : Utiliser le provider mock (tests sans hardware)

**Tâches** :
- `CONFIG_MICRO_ROS_APP_STACK` : Stack size (défaut: `16384` bytes)
- `CONFIG_MICRO_ROS_APP_TASK_PRIO` : Priorité tâche (défaut: `5`)

## 🔌 Ajouter une nouvelle application

Exemple : Publisher IMU

### 1. Créer l'application (`main/uros_app_imu.c`)

```c
#include "uros_core.h"
#include <sensor_msgs/msg/imu.h>

typedef struct {
    sensor_msgs__msg__Imu imu_msg;
    // ... contexte app (buffers, config, etc.)
} uros_app_imu_context_t;

static uros_app_imu_context_t g_imu_context;

static bool imu_app_init(void *app_context) {
    // Init capteur IMU, allouer buffers, etc.
    return true;
}

static bool imu_app_step(void *app_context, void *ros_message) {
    sensor_msgs__msg__Imu *msg = (sensor_msgs__msg__Imu *)ros_message;
    // Lire IMU et remplir msg->linear_acceleration, msg->angular_velocity
    return true;
}

static void imu_app_fini(void *app_context) {
    // Cleanup ressources
}

bool uros_app_imu_start(void) {
    uros_core_config_t config = {
        .node_name = "imu_node",
        .topic_name = "/imu/data",
        .domain_id = 0,
        .timer_period_ms = 50,  // 20 Hz
        .qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        .qos_depth = 10,
        .qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        .qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
        .stack_size = 8192,
        .task_priority = 5,
        .core_affinity = 1,
    };

    uros_app_interface_t app = {
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        .message_size = sizeof(sensor_msgs__msg__Imu),
        .app_init = imu_app_init,
        .app_step = imu_app_step,
        .app_fini = imu_app_fini,
        .app_context = &g_imu_context,
        .app_context_size = sizeof(uros_app_imu_context_t),
    };

    uros_core_context_t *core = uros_core_create(&config, &app);
    if (!core) return false;
    return uros_core_start(core);
}
```

### 2. Mettre à jour `main.c`

```c
#include "uros_app_imu.h"

void app_main(void) {
    // ... init transport USB-CDC ...

    // Lancer l'app IMU au lieu de scan
    uros_app_imu_start();
}
```

### 3. Multi-publisher (optionnel)

```c
// Publier LaserScan ET IMU simultanément
uros_app_scan_start();  // Core 1, task indépendante
uros_app_imu_start();   // Core 1, task indépendante
```

Chaque core a son propre executor, mutex et tasks → indépendants.

## 📚 Documentation détaillée

### Architecture & configuration
- [FR: Architecture, configuration et troubleshooting micro-ROS + ToF](docs/micro_ros_tof_architecture.fr.md)
- [EN: micro-ROS + ToF architecture, configuration, and troubleshooting](docs/micro_ros_tof_architecture.en.md)

### Hardware
- Configuration GPIOs/I2C/IRQ : `main/tof_config.c`
- Mapping bins capteurs : `main/tof_config.c` (table `TOF_HW_CONFIG`)

### Fichiers clés

**Infrastructure micro-ROS** :
- `main/uros_core.c/h` - Core ROS générique
- `main/uros_app_scan.c/h` - Application LaserScan

**Scan ToF** :
- `main/scan_engine.c/h` - Orchestrateur acquisition
- `main/scan_builder.c/h` - Construction message LaserScan
- `main/tof_provider.c/h` - Abstraction hardware ToF
- `main/tof_provider_vl53.c` - Driver VL53L0X
- `main/tof_provider_mock.c` - Générateur mock

**Utilitaires** :
- `main/tof_config.c/h` - Configuration hardware
- `main/tof_snapshot.c/h` - Acquisition snapshot
- `main/led_status.c/h` - LED RGB de status

## 🐛 Troubleshooting

### L'agent ne se connecte pas

```bash
# Vérifier le device USB
ls -l /dev/ttyACM*

# Vérifier les logs ESP32
idf.py monitor
# Rechercher : "Waiting for micro-ROS agent..."
```

### Publication à 0 Hz

- Vérifier QoS : l'agent doit avoir la même QoS (BEST_EFFORT vs RELIABLE)
- Vérifier domain ID : doit matcher entre ESP32 et agent/ROS 2
- Logs : `CONFIG_MICRO_ROS_DEBUG_LOGS=y` dans menuconfig

### Heap overflow / crash

- Augmenter stack size : `CONFIG_MICRO_ROS_APP_STACK` (défaut 16KB)
- Réduire bins : `CONFIG_MICRO_ROS_SCAN_BINS` (défaut 84)

### Publication lente / messages perdus

- Utiliser QoS BEST_EFFORT (défaut)
- Réduire profondeur queue : `CONFIG_MICRO_ROS_QOS_DEPTH`
- Augmenter période : `CONFIG_MICRO_ROS_TIMER_PERIOD_MS`

## 📊 Métriques runtime

Logs toutes les 10s (configurable via `CONFIG_MICRO_ROS_METRICS_LOG_PERIOD_MS`) :

```
runtime metrics: publish_failures=0 scan_step_failures=0 agent_missed_pings=0 congestion_detected=0
```

- `publish_failures` : Échecs `rcl_publish()`
- `scan_step_failures` : Échecs `scan_engine_step()`
- `agent_missed_pings` : Pings agent ratés (3 → restart session)
- `congestion_detected` : Congestion détectée (backoff activé)

## 📝 License

MIT License - Voir [LICENSE](LICENSE)

## 🤝 Contribution

Les pull requests sont bienvenues! Pour des changements majeurs :
1. Ouvrir une issue pour discussion
2. Fork + créer une branche feature
3. Tester sur hardware (si possible)
4. Créer la PR avec description détaillée
