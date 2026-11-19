package org.paraguardian.companion

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import java.util.UUID

object GattProfile {
    private fun uuid(part: Int) = UUID.fromString("5d124b%02x-72b1-419f-b8f7-5d477dc613cb".format(part))
    val SERVICE = uuid(0)
    val SOS = uuid(1)
    val STATUS = uuid(2)
    val PHONE = uuid(3)
    val IDENTITY = uuid(4)
    val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
}

fun Context.granted(permission: String) = checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED
fun Context.canConnect() = Build.VERSION.SDK_INT < 31 || granted(Manifest.permission.BLUETOOTH_CONNECT)

enum class LinkState(val label: String) {
    INACTIVE("Monitoring inactive"), PERMISSION("Bluetooth permission required"),
    DISABLED("Bluetooth off or unavailable"), BACKOFF("Disconnected · retry scheduled"),
    CONNECTING("Connecting"), DISCOVERING("Configuring GATT"), UNVERIFIED("GATT connected · peer unverified"),
    READY("Authenticated device connected")
}

/** Must return a device identity only after provisioning and authenticated verification.
 * The shipping construction supplies no trust verifier while the module backend is unavailable.
 */
fun interface PhysicalTrust {
    fun verifiedPeer(gatt: BluetoothGatt): Peer?
}

@SuppressLint("MissingPermission")
class BluetoothLink(
    private val context: Context,
    private val trust: PhysicalTrust,
    private val state: (LinkState) -> Unit,
    private val packet: (Peer, ByteArray) -> Unit
) {
    private val handler = Handler(Looper.getMainLooper())
    private val adapter: BluetoothAdapter? get() = context.getSystemService(BluetoothManager::class.java)?.adapter
    private var active = false
    private var address: String? = null
    private var connection: BluetoothGatt? = null
    private var peer: Peer? = null
    private val subscriptions = ArrayDeque<BluetoothGattCharacteristic>()
    private val writes = ArrayDeque<ByteArray>()
    private var writing = false
    private var subscribing: UUID? = null
    private var attempt = 0
    var attPayload = 20
        private set

    private val reconnect = Runnable { connect() }
    private val timer = Runnable { fail() }
    private val healthCheck = object : Runnable {
        override fun run() {
            if (!active) return
            if (!context.canConnect() || adapter?.isEnabled != true) {
                close()
                state(if (context.canConnect()) LinkState.DISABLED else LinkState.PERMISSION)
            } else if (connection == null && !reconnectScheduled) {
                connect()
            }
            handler.postDelayed(this, 2_000)
        }
    }
    private var reconnectScheduled = false


    fun start(address: String) {
        stop()
        this.address = address
        active = true
        attempt = 0
        connect()
        handler.postDelayed(healthCheck, 2_000)
    }

    fun stop() {
        active = false
        handler.removeCallbacksAndMessages(null)
        reconnectScheduled = false
        close()
        state(LinkState.INACTIVE)
    }

    fun send(packets: List<ByteArray>): Boolean {
        if (peer == null || connection == null || writes.size + packets.size > 32) return false
        writes.addAll(packets)
        if (!writing) nextWrite()
        return true
    }

    private fun connect() {
        reconnectScheduled = false
        if (!active || connection != null) return
        if (!context.canConnect()) { state(LinkState.PERMISSION); return }
        val bluetooth = adapter
        if (bluetooth?.isEnabled != true) { state(LinkState.DISABLED); return }
        val selected = address ?: return
        try {
            state(LinkState.CONNECTING)
            connection = bluetooth.getRemoteDevice(selected).connectGatt(
                context, false, callbacks, BluetoothDevice.TRANSPORT_LE
            )
            if (connection == null) fail() else deadline()
        } catch (_: IllegalArgumentException) {
            fail()
        } catch (_: SecurityException) {
            fail()
        }
    }

    private fun deadline() {
        handler.removeCallbacks(timer)
        handler.postDelayed(timer, 12_000)
    }

    private fun close() {
        handler.removeCallbacks(timer)
        val gatt = connection
        connection = null
        peer = null
        subscriptions.clear()
        writes.clear()
        writing = false
        subscribing = null
        attPayload = 20
        try { gatt?.close() } catch (_: SecurityException) { }
    }

    private fun fail() {
        close()
        if (!active) return
        state(LinkState.BACKOFF)
        handler.removeCallbacks(reconnect)
        reconnectScheduled = true
        val delay = (1_000L shl attempt.coerceAtMost(6)).coerceAtMost(60_000)
        attempt = (attempt + 1).coerceAtMost(6)
        handler.postDelayed(reconnect, delay)
    }

    private fun BluetoothGatt.runIfCurrent(action: BluetoothGatt.() -> Unit) {
        handler.post {
            if (!active || connection !== this) return@post
            try { action() } catch (_: SecurityException) { fail() }
        }
    }

    private fun BluetoothGatt.discover() {
        if (!discoverServices()) fail() else deadline()
    }

    private fun subscribe(gatt: BluetoothGatt) {
        val characteristic = subscriptions.removeFirstOrNull()
        if (characteristic == null) {
            handler.removeCallbacks(timer)
            attempt = 0
            val verified = trust.verifiedPeer(gatt)
            peer = verified?.takeIf { it.verified && it.route == Route.SMS }
            state(if (peer == null) LinkState.UNVERIFIED else LinkState.READY)
            return
        }
        val descriptor = characteristic.getDescriptor(GattProfile.CCCD)
        if (descriptor == null || !gatt.setCharacteristicNotification(characteristic, true)) { fail(); return }
        val enable = if (characteristic.uuid == GattProfile.SOS) BluetoothGattDescriptor.ENABLE_INDICATION_VALUE
                     else BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        subscribing = characteristic.uuid
        val queued = if (Build.VERSION.SDK_INT >= 33)
            gatt.writeDescriptor(descriptor, enable) == BluetoothStatusCodes.SUCCESS
        else {
            @Suppress("DEPRECATION")
            descriptor.value = enable
            @Suppress("DEPRECATION")
            gatt.writeDescriptor(descriptor)
        }
        if (!queued) fail() else deadline()
    }

    private fun nextWrite() {
        val gatt = connection ?: return
        val bytes = writes.removeFirstOrNull() ?: return
        val characteristic = gatt.getService(GattProfile.SERVICE)?.getCharacteristic(GattProfile.PHONE)
        if (characteristic == null) { fail(); return }
        try {
            val queued = if (Build.VERSION.SDK_INT >= 33) {
                gatt.writeCharacteristic(characteristic, bytes, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) ==
                    BluetoothStatusCodes.SUCCESS
            } else {
                @Suppress("DEPRECATION")
                characteristic.value = bytes
                characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                @Suppress("DEPRECATION")
                gatt.writeCharacteristic(characteristic)
            }
            if (!queued) fail()
            else {
                writing = true
                deadline()
            }
        } catch (_: SecurityException) { fail() }
    }

    private val callbacks = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) = gatt.runIfCurrent {
            if (status != BluetoothGatt.GATT_SUCCESS || newState == BluetoothProfile.STATE_DISCONNECTED) fail()
            else if (newState == BluetoothProfile.STATE_CONNECTED) {
                state(LinkState.DISCOVERING)
                if (!requestMtu(247)) discover() else deadline()
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) = gatt.runIfCurrent {
            if (status == BluetoothGatt.GATT_SUCCESS) attPayload = (mtu - 3).coerceIn(20, 118)
            discover()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) = gatt.runIfCurrent {
            val service = getService(GattProfile.SERVICE)
            val sos = service?.getCharacteristic(GattProfile.SOS)
            val info = service?.getCharacteristic(GattProfile.STATUS)
            val phone = service?.getCharacteristic(GattProfile.PHONE)
            if (status != BluetoothGatt.GATT_SUCCESS || sos == null || info == null || phone == null ||
                sos.properties and BluetoothGattCharacteristic.PROPERTY_INDICATE == 0 ||
                info.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY == 0 ||
                phone.properties and BluetoothGattCharacteristic.PROPERTY_WRITE == 0
            ) fail() else {
                subscriptions.add(sos)
                subscriptions.add(info)
                subscribe(this)
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) =
            gatt.runIfCurrent {
                if (status != BluetoothGatt.GATT_SUCCESS || subscribing != descriptor.characteristic.uuid) fail()
                else { subscribing = null; subscribe(this) }
            }

        override fun onCharacteristicWrite(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int) =
            gatt.runIfCurrent {
                if (status != BluetoothGatt.GATT_SUCCESS || !writing || characteristic.uuid != GattProfile.PHONE) fail()
                else {
                    writing = false
                    handler.removeCallbacks(timer)
                    nextWrite()
                }
            }

        @Deprecated("Used on Android 12 and earlier")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, c: BluetoothGattCharacteristic) {
            changed(gatt, c.uuid, c.value.copyOf())
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            changed(gatt, c.uuid, value.copyOf())
        }
    }

    private fun changed(gatt: BluetoothGatt, uuid: UUID, value: ByteArray) = gatt.runIfCurrent {
        val verified = peer ?: return@runIfCurrent
        if (uuid in setOf(GattProfile.SOS, GattProfile.STATUS) && value.size <= attPayload) packet(verified, value)
    }
}
