package org.paraguardian.companion

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationManager
import android.os.Build
import android.os.CancellationSignal
import android.os.SystemClock
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withTimeoutOrNull
import kotlin.coroutines.resume

@SuppressLint("MissingPermission")
suspend fun visiblePhoneLocation(context: Context, activeIncident: Boolean): Position? {
    if (!context.granted(Manifest.permission.ACCESS_COARSE_LOCATION) &&
        !context.granted(Manifest.permission.ACCESS_FINE_LOCATION)) return null
    val manager = context.getSystemService(LocationManager::class.java)
    val providers = listOf(LocationManager.GPS_PROVIDER, LocationManager.NETWORK_PROVIDER)
    val suitable = providers.filter { it != LocationManager.GPS_PROVIDER ||
        context.granted(Manifest.permission.ACCESS_FINE_LOCATION) }
    val provider = suitable.firstOrNull { runCatching { manager.isProviderEnabled(it) }.getOrDefault(false) }
    val current = if (provider != null && Build.VERSION.SDK_INT >= 30) withTimeoutOrNull(3_000) {
        suspendCancellableCoroutine<Location?> { continuation ->
            val signal = CancellationSignal()
            continuation.invokeOnCancellation { signal.cancel() }
            try {
                manager.getCurrentLocation(provider, signal, context.mainExecutor) {
                    if (continuation.isActive) continuation.resume(it)
                }
            } catch (_: RuntimeException) {
                if (continuation.isActive) continuation.resume(null)
            }
        }
    } else null
    val fix = current ?: suitable.mapNotNull {
        runCatching { manager.getLastKnownLocation(it) }.getOrNull()
    }.maxByOrNull { it.elapsedRealtimeNanos } ?: return null
    if (fix.latitude !in -90.0..90.0 || fix.longitude !in -180.0..180.0) return null
    val age = (SystemClock.elapsedRealtimeNanos() - fix.elapsedRealtimeNanos) / 1_000_000
    if (age !in 0..(24L * 60 * 60 * 1000)) return null
    return Position(fix.latitude, fix.longitude, age, fix.accuracy.toDouble().takeIf { fix.hasAccuracy() },
        "Phone location", current == null || age > 60_000, current != null && activeIncident,
        System.currentTimeMillis() - age)
}
