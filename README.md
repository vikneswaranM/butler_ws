# Butler Bot Food Delivery System

A complete ROS 2 implementation of an autonomous food delivery robot with advanced state machine logic, timeout handling, and order cancellation support.

## 📋 Overview

This system enables a butler robot to autonomously deliver food from a kitchen to customer tables in a café environment. It handles 7 different delivery scenarios including confirmations, timeouts, cancellations, and multi-table orders.

## 🎯 Features

- ✅ **Location-based Navigation**: Navigate using location names (kitchen, table_1, etc.)
- ✅ **Confirmation System**: Wait for confirmations at kitchen and tables with configurable timeouts
- ✅ **Order Cancellation**: Cancel entire orders or specific tables during delivery
- ✅ **Multi-table Support**: Deliver to multiple tables in a single order
- ✅ **Smart Routing**: Intelligently returns via kitchen when needed
- ✅ **State Machine**: Robust 8-state state machine for complex workflows
- ✅ **Real-time Feedback**: Continuous status updates during delivery
- ✅ **Thread-safe**: Handles concurrent confirmations and cancellations

## 🚀 Quick Start

### Prerequisites

1. ROS 2 Humble installed
2. Nav2 navigation stack
3. Butler bot simulation/hardware setup
4. Location file: `~/Documents/location_for_butler_bot`

### Build

```bash
cd ~/xprj/butler_ws
colcon build --packages-select butler_bot
source install/setup.bash
```

### Run

**Terminal 1: Launch Navigation**
```bash
ros2 launch butler_bot localization_launch.py
```

**Terminal 2: Launch Food Delivery System**
```bash
ros2 launch butler_bot food_delivery.launch.py
```

**Terminal 3: Send Order**
```bash
# Simple delivery to table 1
ros2 run butler_bot food_delivery_client.py 1

# Multiple tables with 30s timeout
ros2 run butler_bot food_delivery_client.py 1,2,3 30
```

**Terminal 4: Interact (Optional)**
```bash
# Confirm at location
ros2 service call /confirm_location butler_bot/srv/Confirmation "{location: 'kitchen'}"

# Cancel order
ros2 service call /cancel_order butler_bot/srv/CancelOrder "{table_number: 0}"
```

## 📚 Documentation

### User Documentation
- **[FOOD_DELIVERY_GUIDE.md](FOOD_DELIVERY_GUIDE.md)** - Complete user guide with all 7 scenarios explained

### Technical Documentation
- **[TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md)** - Comprehensive technical documentation including:
  - System architecture and design decisions
  - Code implementation details
  - State machine design
  - API reference
  - Testing procedures
  - Issues and resolutions

## 🎬 Scenarios Implemented

| # | Scenario | Description |
|---|----------|-------------|
| 1 | Simple Delivery | Home → Kitchen → Table → Home (automatic) |
| 2 | With Confirmation | Wait for confirmations at each location |
| 3a | Kitchen Timeout | No kitchen confirmation → return home |
| 3b | Table Timeout | Kitchen confirmed, table timeout → kitchen → home |
| 4a | Cancel at Table | Cancelled going to table → kitchen → home |
| 4b | Cancel at Kitchen | Cancelled going to kitchen → home |
| 5 | Multiple Tables | Deliver to multiple tables automatically |
| 6 | Multi-table Timeout | Skip timed-out tables, continue others |
| 7 | Multi-table Cancel | Skip cancelled tables, deliver to others |

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│     Food Delivery Client                │
│     (food_delivery_client.py)           │
└────────────────┬────────────────────────┘
                 │ FoodDelivery Action
                 ▼
┌─────────────────────────────────────────┐
│     Food Delivery Robot                 │
│     (food_delivery_robot.py)            │
│     - State Machine                     │
│     - Order Management                  │
│     - Confirmation/Cancel Services      │
└────────────────┬────────────────────────┘
                 │ NavigateToLocation Action
                 ▼
┌─────────────────────────────────────────┐
│     Navigate To Location Server         │
│     (navigate_to_location_server.py)    │
│     - Location Management               │
│     - Nav2 Integration                  │
└────────────────┬────────────────────────┘
                 │ NavigateToPose Action
                 ▼
┌─────────────────────────────────────────┐
│     Nav2 Stack                          │
└─────────────────────────────────────────┘
```

## 📦 Components

### Actions
- **FoodDelivery.action** - Main delivery workflow
- **NavigateToLocation.action** - Location-based navigation

### Services
- **Confirmation.srv** - Confirm arrival at location
- **CancelOrder.srv** - Cancel order or specific table

### Nodes
- **food_delivery_robot.py** - Main state machine (390 lines)
- **food_delivery_client.py** - Order submission client
- **navigate_to_location_server.py** - Navigation server (377 lines)

## 🧪 Testing

### Run a Test Scenario

```bash
# Scenario 1: Simple delivery
ros2 run butler_bot food_delivery_client.py 1

# Scenario 7: Multi-table with cancellation
ros2 run butler_bot food_delivery_client.py 1,2,3 30
# In another terminal:
ros2 service call /confirm_location butler_bot/srv/Confirmation "{location: 'kitchen'}"
ros2 service call /confirm_location butler_bot/srv/Confirmation "{location: 'table_1'}"
ros2 service call /cancel_order butler_bot/srv/CancelOrder "{table_number: 2}"
ros2 service call /confirm_location butler_bot/srv/Confirmation "{location: 'table_3'}"
```


### Action server not responding
```bash
# Restart food delivery system
ros2 launch butler_bot food_delivery.launch.py
``

## 🔄 Workflow Example

**Scenario 3b: Table timeout after kitchen confirmation**

```
1. Client sends order: table_1, timeout=30s
2. Robot: IDLE → GOING_TO_KITCHEN
3. Robot arrives: AT_KITCHEN
4. Wait 30s for confirmation
5. User confirms kitchen ✓
6. Robot: GOING_TO_TABLE
7. Robot arrives: AT_TABLE
8. Wait 30s for confirmation
9. Timeout! (no confirmation)
10. Robot: RETURNING_TO_KITCHEN (food picked up, return it)
11. Robot: GOING_HOME
12. Robot: AT_HOME → IDLE
13. Result: delivered=[], skipped=[1]
```

## 🚦 State Machine

```
IDLE
  ↓ order received
GOING_TO_KITCHEN
  ↓ reached kitchen
AT_KITCHEN
  ↓ confirmed (or timeout → GOING_HOME)
GOING_TO_TABLE
  ↓ reached table (or cancelled → RETURNING_TO_KITCHEN)
AT_TABLE
  ↓ confirmed/timeout
[Repeat for each table]
  ↓ all done
RETURNING_TO_KITCHEN (if needed)
  ↓
GOING_HOME
  ↓
AT_HOME
  ↓
IDLE
```

## 🎓 Key Technologies

- **ROS 2 Humble** - Robot Operating System
- **Python 3.10** - Implementation language
- **Nav2** - Navigation stack
- **Action Servers** - Asynchronous task execution
- **Services** - Synchronous interactions
- **Multi-threaded Executor** - Concurrent callback handling
- **State Machine Pattern** - Complex behavior management

## 📖 API Quick Reference

### Send Delivery Order
```python
goal = FoodDelivery.Goal()
goal.table_numbers = [1, 2, 3]
goal.confirmation_timeout = 30.0  # 0 = no confirmation
```

### Confirm Location
```bash
ros2 service call /confirm_location butler_bot/srv/Confirmation \
  "{location: 'kitchen'}"
```

### Cancel Order
```bash
# Cancel entire order
ros2 service call /cancel_order butler_bot/srv/CancelOrder \
  "{table_number: 0}"

# Cancel specific table
ros2 service call /cancel_order butler_bot/srv/CancelOrder \
  "{table_number: 2}"
```

## 🔗 Related Documentation

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

---

