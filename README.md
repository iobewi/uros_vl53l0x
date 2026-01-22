# micro-ROS + ToF (VL53L0X/mock)

Ce dépôt contient une application micro-ROS (ESP-IDF) qui publie un `LaserScan`
à partir de capteurs Time-of-Flight (ToF), avec un provider mock ou VL53L0X.

## Documentation

### Documentation principale (architecture & configuration)
- [FR: Architecture, configuration et troubleshooting micro-ROS + ToF](docs/micro_ros_tof_architecture.fr.md)
- [EN: micro-ROS + ToF architecture, configuration, and troubleshooting](docs/micro_ros_tof_architecture.en.md)

### Autres ressources
- Paramétrage des GPIOs/I2C/IRQ, bins, QoS et timing via Kconfig : voir `main/Kconfig.projbuild`.
- Point d’entrée et flux d’exécution : `main/main.c` et `main/micro_ros_adapter.c`.
- Logs embarqués et métriques micro-ROS : voir les sections dédiées dans la documentation.
