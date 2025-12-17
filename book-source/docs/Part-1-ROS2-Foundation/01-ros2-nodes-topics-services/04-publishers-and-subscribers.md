---
sidebar_position: 4
title: "Lesson 4: Services and Synchronous Requests"
description: "Implement synchronous request-response communication using ROS 2 services"
---

# Lesson 4: Services and Synchronous Requests

**Learning Outcome:** Create a service server that responds to client requests and implement error handling.

**Proficiency Level:** B1

**Estimated Time:** 50 minutes

**New Concepts:** 2
- Service: synchronous request-response pattern
- Service client and server architecture

---

## Working Example: Battery Status Service

First, define the service interface. Create a file called `Battery.srv`:

```
---
int32 percentage
string status
int32 timestamp
```

The `---` separator divides request (top) from response (bottom). This service takes no parameters and returns battery information.

**Service Server** (`battery_service_server.py`):

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import GetParameterTypes
import time
import random


class BatteryStatusService(Node):
    def __init__(self):
        super().__init__('battery_service_server')
        # Create a simple service using GetParameterTypes as placeholder
        self.srv = self.create_service(
            GetParameterTypes,
            'battery_status',
            self.handle_battery_request)
        self.get_logger().info('Battery service server started')
        self.battery = 100

    def handle_battery_request(self, request, response):
        # Simulate battery declining
        self.battery = max(0, self.battery - random.randint(1, 5))

        # Build response
        status = 'charging' if self.battery > 80 else 'discharging'
        self.get_logger().info(f'Battery query: {self.battery}% - {status}')

        # For this example, we'll just log
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Service Client** (`battery_service_client.py`):

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import GetParameterTypes
import sys


class BatteryServiceClient(Node):
    def __init__(self):
        super().__init__('battery_service_client')
        self.cli = self.create_client(GetParameterTypes, 'battery_status')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Battery service not available, waiting...')

        self.get_logger().info('Battery service available')

    def send_request(self):
        request = GetParameterTypes.Request()

        # Send request (blocking call)
        try:
            future = self.cli.call_async(request)
            self.get_logger().info('Request sent, waiting for response...')
            # In real implementation, you'd handle the future properly
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryServiceClient()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Understanding Services

A **service** is different from a topic. It's **synchronous request-response** communication:

**Topic (Pub/Sub):**
```
Publisher --message--> Topic --message--> Subscriber
           (doesn't wait)        (gets data when ready)
```

**Service:**
```
Client --request--> Service --response--> Client waits
                    (blocks)
```

### When to Use Each

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topic** | One-way streaming | Sensor readings (temperature continuously) |
| **Service** | Request-response | Query battery level (ask and wait for answer) |

### Service Definition Format

```
# Request section (what client sends)
float64 temperature
int32 id

---

# Response section (what server sends back)
string status
int32 code
```

Everything before `---` is the request. Everything after is the response.

### Breaking Down the Server

**Create a Service:**
```python
self.srv = self.create_service(
    GetParameterTypes,  # Service type
    'battery_status',   # Service name
    self.handle_battery_request)  # Callback function
```

**Handle Requests:**
```python
def handle_battery_request(self, request, response):
    # Process request
    status = 'charging' if self.battery > 80 else 'discharging'

    # Build response
    response.percentage = self.battery
    response.status = status

    return response  # Send back to client
```

### Breaking Down the Client

**Create a Client:**
```python
self.cli = self.create_client(GetParameterTypes, 'battery_status')
```

**Wait for Service:**
```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Waiting for service...')
```

A robust client checks that the service exists before calling it.

**Call the Service:**
```python
request = GetParameterTypes.Request()
future = self.cli.call_async(request)  # Non-blocking
# Or use call_sync for blocking behavior
```

---

## Hands-On Practice

### Exercise 4.1: Implement a Status Service

Create a file called `simple_status_server.py` that:
- Creates a service server named 'system_status'
- Responds with a simple acknowledgment message
- Logs each service call

**Success Criteria:**
- [ ] Server starts without errors
- [ ] Server logs indicate it's ready
- [ ] Service name is 'system_status'

### Exercise 4.2: Create a Service Client

Create a file called `simple_status_client.py` that:
- Waits for the 'system_status' service
- Sends a request
- Logs the response

**Success Criteria:**
- [ ] Client finds the service
- [ ] Client sends request successfully
- [ ] Response received from server

### Exercise 4.3: Add Error Handling

Modify the client to handle timeouts:
```python
if not self.cli.wait_for_service(timeout_sec=2.0):
    self.get_logger().error('Service not available')
    return
```

Test by:
1. Starting the client WITHOUT the server
2. Observe timeout error
3. Start the server
4. Run client again, see success

**Success Criteria:**
- [ ] Client handles missing service gracefully
- [ ] Clear error message when service unavailable
- [ ] Works when service becomes available

---

## Key Differences: Topics vs Services

| Aspect | Topic | Service |
|--------|-------|---------|
| **Communication** | Asynchronous | Synchronous |
| **Wait** | Publisher doesn't wait | Client waits for response |
| **Multiple Subs** | Yes | No (one client per call) |
| **Use Case** | Continuous data stream | Query, ask for computation |
| **Decoupling** | Highly decoupled | Tightly coupled |

---

## Try With AI

Explore service design patterns:

**Step 1: Ask about service vs topic tradeoffs**
Prompt your AI:
```
When designing a robot system, when would you use:
1. A topic to communicate navigation commands?
2. A service to query the robot's current position?

What are the pros and cons of each approach for these scenarios?
```

**Step 2: Review and compare**
Ask yourself:
- Is navigation (ongoing commands) better as topic or service?
- Is position (point query) better as topic or service?
- What happens if you use the wrong pattern?

**Step 3: Think about timeouts**
Prompt your AI:
```
What should happen in a robot system if:
1. A client calls a service but the server takes 10 seconds to respond?
2. A client calls a service but the server is crashed?
3. A service is called 1000 times per second?

How does ROS 2 handle these scenarios?
```

**Step 4: Apply to robotics**
Consider a real robot with:
- Sensor topics (continuous stream)
- Motor command service (request immediate action)
- Battery query service (ask for status)

Which makes sense for each? Why?

**Expected Outcome:** Understanding when to choose synchronous (service) vs asynchronous (topic) communication for different robot tasks.
