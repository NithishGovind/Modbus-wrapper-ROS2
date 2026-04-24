import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray, UInt8MultiArray

from ros2_modbus.srv import (
    ReadCoils,
    ReadRegisters,
    WriteCoil,
    WriteRegister,
)


class ModbusClient(Node):
    def __init__(self, plc_ip: str = '192.168.1.88', plc_port: int = 502):
        super().__init__('modbus_client')
        self.plc_ip = plc_ip
        self.plc_port = plc_port
        self.get_logger().info(
            f'modbus_client started (target {self.plc_ip}:{self.plc_port})')

        self.reg_sub = self.create_subscription(
            UInt16MultiArray, '/modbus/registers', self._on_registers, 10)
        self.coil_sub = self.create_subscription(
            UInt8MultiArray, '/modbus/coils', self._on_coils, 10)

        self.cli_read_regs = self.create_client(ReadRegisters, '/modbus/read_registers')
        self.cli_write_reg = self.create_client(WriteRegister, '/modbus/write_register')
        self.cli_read_coils = self.create_client(ReadCoils, '/modbus/read_coils')
        self.cli_write_coil = self.create_client(WriteCoil, '/modbus/write_coil')

        self._demo_done = False
        self._startup_timer = self.create_timer(2.0, self._run_demo_once)

    def _on_registers(self, msg: UInt16MultiArray) -> None:
        self.get_logger().info(f'registers: {list(msg.data)}')

    def _on_coils(self, msg: UInt8MultiArray) -> None:
        self.get_logger().info(f'coils: {list(msg.data)}')

    def _run_demo_once(self) -> None:
        if self._demo_done:
            return
        self._demo_done = True
        self._startup_timer.cancel()

        for name, cli in (
            ('read_registers', self.cli_read_regs),
            ('write_register', self.cli_write_reg),
            ('read_coils', self.cli_read_coils),
            ('write_coil', self.cli_write_coil),
        ):
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'service {name} not available')
                return

        self._call_async(
            self.cli_read_regs,
            ReadRegisters.Request(address=0, count=10),
            'read_registers(0, 10)',
        )
        self._call_async(
            self.cli_write_reg,
            WriteRegister.Request(address=0, value=1234),
            'write_register(0, 1234)',
        )
        self._call_async(
            self.cli_read_coils,
            ReadCoils.Request(address=0, count=8),
            'read_coils(0, 8)',
        )
        self._call_async(
            self.cli_write_coil,
            WriteCoil.Request(address=0, value=True),
            'write_coil(0, True)',
        )

    def _call_async(self, client, request, label: str) -> None:
        future = client.call_async(request)

        def _done(fut):
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'{label} exception: {exc}')
                return
            if resp is None:
                self.get_logger().error(f'{label} returned None')
                return
            if hasattr(resp, 'values'):
                self.get_logger().info(
                    f'{label} -> success={resp.success} msg="{resp.message}" '
                    f'values={list(resp.values)}')
            else:
                self.get_logger().info(
                    f'{label} -> success={resp.success} msg="{resp.message}"')

        future.add_done_callback(_done)


def main(args=None):
    rclpy.init(args=args)
    node = ModbusClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
