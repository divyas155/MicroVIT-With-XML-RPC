# Sequence Diagrams

## Image Processing and Alert Generation

```
Nano (ROS1)              Orin (ROS2)              MQTT Broker
   |                         |                         |
   |--[Capture Image]------->|                         |
   |                         |                         |
   |                         |--[MicroViT Process]--->|
   |                         |<--[Features]-----------|
   |                         |                         |
   |                         |--[Ollama Generate]---->|
   |                         |<--[AI Message]---------|
   |                         |                         |
   |                         |--[Publish Alert]------>|
   |                         |                         |
```

## Command Flow

```
Controller              MQTT Broker              Robot1 (Orin)         Robot1 (Nano)
   |                         |                         |                      |
   |--[Analyze Telemetry]--->|                         |                      |
   |<--[Telemetry]-----------|                         |                      |
   |                         |                         |                      |
   |--[Generate Command]---->|                         |                      |
   |                         |--[Command]------------>|                      |
   |                         |                         |                      |
   |                         |                         |--[XML-RPC Cmd]----->|
   |                         |                         |                      |
   |                         |                         |<--[Status]----------|
   |                         |                         |                      |
   |                         |<--[Status Update]-------|                      |
```

## Complete Workflow

```
Time    Nano          Orin           Controller      Helper Robot    MQTT Broker
  |       |             |                 |                |              |
  |       |--[Image]--->|                 |                |              |
  |       |             |--[Process]----->|                |              |
  |       |             |                 |                |              |
  |       |             |<--[Features]----|                |              |
  |       |             |                 |                |              |
  |       |             |--[Generate]---->|                |              |
  |       |             |                 |                |              |
  |       |             |--[Alert]------->|                |              |
  |       |             |                 |                |              |
  |       |             |                 |--[Analyze]---->|              |
  |       |             |                 |                |              |
  |       |             |                 |<--[Decision]---|              |
  |       |             |                 |                |              |
  |       |             |                 |--[Command]----->|              |
  |       |             |                 |                |              |
  |       |             |<--[Command]-----|                |              |
  |       |             |                 |                |              |
  |       |<--[Cmd]-----|                 |                |              |
  |       |             |                 |                |              |
  |       |--[Move]---->|                 |                |              |
  |       |             |                 |                |              |
  |       |             |                 |--[Task]-------->|              |
  |       |             |                 |                |              |
  |       |             |                 |                |--[Execute]--->|
  |       |             |                 |                |              |
```

## Error Handling Flow

```
Component           Error Handler          Log System        MQTT Broker
   |                      |                    |                  |
   |--[Error]------------>|                    |                  |
   |                      |                    |                  |
   |                      |--[Log Error]------>|                  |
   |                      |                    |                  |
   |                      |--[Retry]---------->|                  |
   |                      |                    |                  |
   |<--[Retry Result]-----|                    |                  |
   |                      |                    |                  |
   |--[Success/Fail]----->|                    |                  |
   |                      |                    |                  |
   |                      |--[Alert]---------->|                  |
```
