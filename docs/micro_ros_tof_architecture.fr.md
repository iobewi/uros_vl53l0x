# micro-ROS + ToF (mock/VL53) — architecture, configuration, flux d’exécution

[English version](micro_ros_tof_architecture.en.md)

Ce document central décrit le pipeline de démarrage, la sélection du provider ToF (mock/VL53),
les paramètres Kconfig critiques, la QoS/temporalité ROS, et les diagnostics d’erreurs courantes.

## 1) Vue d’ensemble / Architecture

### Pipeline de démarrage (flux principal)

```
app_main
 ├─ init transport USB-CDC (micro-ROS custom transport)
 ├─ tof_provider_init()
 ├─ micro_ros_adapter_start()
 │   └─ micro_ros_task
 │       ├─ attente agent micro-ROS
 │       ├─ init RCL, node, publisher, timer, executor
 │       ├─ scan_engine_init + scan_builder_init
 │       ├─ boucle: timer → scan_pub_task → scan_engine_step → rcl_publish
 │       └─ surveillance agent + relance
 └─ boucle FreeRTOS (tâches)
```

**Emplacements clefs**
- Démarrage global et transport USB-CDC dans `app_main()`.【F:main/main.c†L1-L61】
- L’app micro-ROS est démarrée par `uros_app_start()` → `micro_ros_adapter_start()`.【F:main/uros_app.c†L1-L9】【F:main/micro_ros_adapter.c†L570-L605】
- Le pipeline scan est assuré par `scan_engine_step()` → `tof_provider_snapshot()` → `scan_builder_fill()`.
  【F:main/scan_engine.c†L120-L129】【F:main/scan_builder.c†L156-L186】【F:main/tof_provider.h†L24-L29】

### Rôle des modules

- **tof_provider_(mock|vl53).c**
  - Fournit des échantillons ToF (range + status + valid) via `tof_provider_snapshot()`.
  - La version mock génère des signaux synthétiques (sinusoïde, “cliff”, bursts invalides).
    【F:main/tof_provider_mock.c†L108-L172】
  - La version VL53L0X gère l’init I2C, l’assignation d’adresses par XSHUT, et les IRQ data-ready.
    【F:main/tof_provider_vl53.c†L220-L321】

- **scan_engine.c**
  - Orchestration du scan : snapshot ToF → construction LaserScan → timestamp.
  - Valide la cohérence du mapping capteur→bin et applique la source de temps.
    【F:main/scan_engine.c†L29-L108】【F:main/scan_engine.c†L120-L139】

- **scan_builder.c**
  - Construit le message `sensor_msgs/LaserScan` (ranges, frame_id, angles, limites).
  - Ignore les samples invalides et ne remplit que les bins valides.
    【F:main/scan_builder.c†L43-L123】【F:main/scan_builder.c†L156-L186】

- **micro_ros_adapter.c**
  - Démarre la session micro-ROS (node, publisher, timer, executor) et publie le LaserScan.
  - Gère la détection de l’agent, la QoS, la synchro temps, les backoffs d’erreurs.
    【F:main/micro_ros_adapter.c†L286-L520】

- **led_status.c**
  - Indicateur d’état (attente agent, connecté, erreur) via LED configurable.
    【F:main/led_status.c†L43-L133】

## 2) Configuration/Kconfig critique

### micro-ROS (publishing & scheduling)
- `CONFIG_MICRO_ROS_DOMAIN_ID`: domaine micro-ROS pour la découverte des nodes.【F:main/Kconfig.projbuild†L15-L19】
- `CONFIG_MICRO_ROS_NODE_NAME`: nom du node publishant le LaserScan.【F:main/Kconfig.projbuild†L21-L25】
- `CONFIG_MICRO_ROS_TOPIC_NAME`: topic LaserScan publié.【F:main/Kconfig.projbuild†L27-L31】
- `CONFIG_MICRO_ROS_TIMER_PERIOD_MS`: période de publication (ms).【F:main/Kconfig.projbuild†L33-L37】
- `CONFIG_MICRO_ROS_QOS_RELIABLE`: bascule QoS RELIABLE vs BEST_EFFORT.【F:main/Kconfig.projbuild†L39-L45】
- `CONFIG_MICRO_ROS_QOS_DEPTH`: profondeur QoS (KEEP_LAST) pour le LaserScan.【F:main/Kconfig.projbuild†L47-L52】
- `CONFIG_MICRO_ROS_SCAN_BINS`: nombre de bins LaserScan (résolution angulaire).【F:main/Kconfig.projbuild†L54-L58】
- `CONFIG_MICRO_ROS_SCAN_FRAME_ID`: frame_id du LaserScan.【F:main/Kconfig.projbuild†L60-L64】
- Logs/diagnostics :
  - `CONFIG_MICRO_ROS_PUBLISH_LOG_DIVIDER`, `CONFIG_MICRO_ROS_PUBLISH_ERROR_LOG_DIVIDER`,
    `CONFIG_MICRO_ROS_SCAN_STEP_ERROR_LOG_DIVIDER`, `CONFIG_MICRO_ROS_EXECUTOR_ERROR_LOG_DIVIDER`.
    【F:main/Kconfig.projbuild†L72-L98】
  - `CONFIG_MICRO_ROS_METRICS_LOG_PERIOD_MS`: période d’affichage des métriques runtime (ms).
    【F:main/Kconfig.projbuild†L99-L105】
- Gestion mémoire scan :
  - `CONFIG_MICRO_ROS_SCAN_ALLOC_GUARD`, `CONFIG_MICRO_ROS_SCAN_BUILDER_ALLOC_MALLOC`.
    【F:main/Kconfig.projbuild†L107-L120】

### ToF (I2C, GPIOs, mapping)
- `CONFIG_TOF_COUNT`: nombre de capteurs, indexés de 0 à N-1.【F:main/Kconfig.projbuild†L140-L146】
- `CONFIG_TOF_BIN_ALLOW_DUPLICATES`: autorise ou non les collisions de bins.
  (désactivé = détection d’indices dupliqués dans `scan_engine`).【F:main/Kconfig.projbuild†L148-L152】【F:main/scan_engine.c†L37-L63】
- Bus I2C :
  - `CONFIG_TOF_I2C_SDA_GPIO`, `CONFIG_TOF_I2C_SCL_GPIO`, `CONFIG_TOF_I2C_FREQ_HZ`.
    【F:main/Kconfig.projbuild†L154-L170】
- Timing/IRQ :
  - `CONFIG_TOF_TIMING_BUDGET_US`, `CONFIG_TOF_GPIO_READY_TIMEOUT_MS`.
    【F:main/Kconfig.projbuild†L172-L182】
- Par capteur (`TOF_0_*`, `TOF_1_*`, …) :
  - `*_XSHUT_GPIO`, `*_INT_GPIO`, `*_ADDR_7B`, `*_BIN_IDX`.
    【F:main/Kconfig.projbuild†L184-L336】

### LED statut micro-ROS
- `CONFIG_MICRO_ROS_STATUS_LED_ENABLE`: active l’indicateur d’état LED.【F:main/Kconfig.projbuild†L115-L119】
- `CONFIG_MICRO_ROS_STATUS_LED_GPIO`: GPIO de la LED.【F:main/Kconfig.projbuild†L121-L126】
- `CONFIG_MICRO_ROS_STATUS_LED_BRIGHTNESS`: niveau de luminosité (0-255).【F:main/Kconfig.projbuild†L128-L134】

### Sélection du provider ToF
- `CONFIG_TOF_PROVIDER_MOCK`: sélectionne le provider mock (sinon VL53L0X).
  Utilisé dans le CMake pour choisir la bonne implémentation à la compilation.
  【F:main/Kconfig.projbuild†L342-L349】【F:main/CMakeLists.txt†L10-L16】

## 3) QoS & timing ROS

- **QoS LaserScan**
  - KEEP_LAST depth=`CONFIG_MICRO_ROS_QOS_DEPTH`.
  - RELIABLE ou BEST_EFFORT selon `CONFIG_MICRO_ROS_QOS_RELIABLE`.
  - DURABILITY = VOLATILE pour éviter la pression XRCE.
  【F:main/micro_ros_adapter.c†L342-L367】

- **Période de publication**
  - Timer `CONFIG_MICRO_ROS_TIMER_PERIOD_MS` déclenche `scan_pub_task`.
  【F:main/micro_ros_adapter.c†L333-L414】

- **Paramètres LaserScan dérivés**
  - `scan_time = period_ms / 1000.0`, `bins = CONFIG_MICRO_ROS_SCAN_BINS`.
  - `angle_min = -π`, `angle_increment = 2π / bins`.
  - `range_min = 0.03m`, `range_max = 2.0m`.
  【F:main/micro_ros_adapter.c†L47-L58】【F:main/micro_ros_adapter.c†L286-L294】

- **Timestamp**
  - `rmw_uros_epoch_nanos()` si la synchro est OK.
  - Fallback `esp_timer_get_time()` (log warning) si sync indisponible.
  【F:main/micro_ros_adapter.c†L383-L383】【F:main/scan_engine.c†L66-L90】

## 4) Provider ToF

### Mock (simulation)
- Génère des ranges sinusoidales et un “cliff” (grande distance) qui tourne.
- Injecte des bursts invalides (status=4, range=NaN) sur un capteur tournant.
- Mise à jour à 50 Hz (delay 20 ms).
  【F:main/tof_provider_mock.c†L117-L173】

### VL53L0X (réel)
- Init I2C + timing budget via `vl53l0x_i2c_master_init()`.
- Assignation d’adresses par XSHUT (`vl53l0x_multi_assign_addresses`).
- IRQ data-ready sur GPIO, lecture protégée par mutex I2C, puis clear interrupt.
- Tâche dédiée par capteur, avec backoff exponentiel sur erreurs (GPIO/I2C).
  【F:main/tof_provider_vl53.c†L220-L361】【F:main/tof_provider_vl53.c†L138-L209】

### Interprétation des statuts ToF
- `status = 0` : mesure valide (VL53 RangeStatus OK).
- `status = 250` : timeout prise mutex I2C (lecture bloquée).
- `status = 251` : timeout attente GPIO data-ready.
- `status = 252` : timeout snapshot (lecture lock-free en dépassement temps/spin).
- `status = 255` : état invalide générique (init/erreur non catégorisée).
  【F:main/tof_provider_vl53.c†L138-L209】【F:main/tof_provider_mock.c†L9-L16】【F:main/tof_provider_vl53.c†L9-L16】

## 5) Diagnostics / Erreurs courantes (Troubleshooting)

### Perte de l’agent micro-ROS
- La tâche attend l’agent puis init RCL.
- Si `rmw_uros_ping_agent()` échoue trop souvent, la session est relancée.
  【F:main/micro_ros_adapter.c†L316-L517】

**Symptômes**
- Logs : “micro-ROS agent lost, restarting session”.
- LED clignotante en mode attente.

**Actions**
- Vérifier le transport USB-CDC et que l’agent est lancé.
- Vérifier le `CONFIG_MICRO_ROS_DOMAIN_ID` côté agent et device.
  【F:main/main.c†L17-L41】【F:main/Kconfig.projbuild†L12-L18】

### Time sync indisponible
- Sync time tente 5 fois puis log warning, fallback sur `esp_timer_get_time()`.
  【F:main/micro_ros_adapter.c†L169-L184】【F:main/scan_engine.c†L66-L90】

**Symptômes**
- Logs : “Time sync failed; timestamps may be unset” / fallback timestamp.

**Actions**
- Vérifier l’agent micro-ROS (time sync) et le transport XRCE.

### Snapshot / timeout ToF
- Les snapshots sont lock-free : si le capteur est en update, un timeout renvoie
  `status=252` et `range=NaN`.
  【F:main/tof_provider_mock.c†L81-L129】【F:main/tof_provider_vl53.c†L81-L129】

**Symptômes**
- Logs : “Snapshot timeout (idx=…)”.

**Actions**
- Vérifier les délais I2C/IRQ (ex. `CONFIG_TOF_GPIO_READY_TIMEOUT_MS`).
- Réduire la charge CPU ou augmenter la période de publication.
  【F:main/Kconfig.projbuild†L154-L164】【F:main/Kconfig.projbuild†L36-L42】

### Congestion / échec de publication
- Les erreurs répétées de publication déclenchent un backoff (`publish_backoff_cycles`).
- Une congestion est détectée si l’agent est joignable mais les publications échouent.
  【F:main/micro_ros_adapter.c†L232-L260】【F:main/micro_ros_adapter.c†L480-L505】

**Actions**
- Passer en BEST_EFFORT si RELIABLE provoque du backpressure XRCE.
- Diminuer la fréquence de publication (augmenter `CONFIG_MICRO_ROS_TIMER_PERIOD_MS`).
  【F:main/micro_ros_adapter.c†L342-L367】【F:main/Kconfig.projbuild†L36-L42】

### Erreurs VL53 courantes
- `status=250/251` : timeouts I2C/GPIO (vérifier câblage, IRQ, pull-ups I2C).
- `status=255` : init/mesure non valide (capteur non trouvé ou init ratée).
  【F:main/tof_provider_vl53.c†L138-L209】【F:main/tof_provider_vl53.c†L220-L321】

## 6) Notes pratiques d’intégration

- Le provider ToF est choisi **au build** via `CONFIG_TOF_PROVIDER_MOCK`.
  Aucune bascule runtime n’est prévue, donc ajuster via menuconfig.
  【F:main/Kconfig.projbuild†L313-L321】【F:main/CMakeLists.txt†L10-L16】
- Les bins LaserScan sont mappés par capteur via `TOF_*_BIN_IDX`. Les doublons
  sont refusés si `CONFIG_TOF_BIN_ALLOW_DUPLICATES=n`.
  【F:main/Kconfig.projbuild†L166-L311】【F:main/scan_engine.c†L37-L63】

---

> ✅ Ce document couvre l’architecture, la configuration, les providers ToF,
> la QoS/temporalité ROS, et les procédures de diagnostic pour l’intégration
> et le debug terrain.
