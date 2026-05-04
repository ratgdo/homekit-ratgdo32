/***********************************************************************
 * homekit-ratgdo logger web page javascript functions
 *
 * Copyright (c) 2024-25 David Kerr, https://github.com/dkerr64
 *
 */

// Global vars...
var evtSource = undefined;      // for Server Sent Events (SSE)
var msgJson = undefined;        // for status
// v27: persist the SSE client UUID in localStorage so a page reload
// reuses the same subscription slot instead of leaking the previous one.
// Pre-v27, every reload allocated a fresh UUID + slot; with the firmware
// orphan sweep on a 15s pre-handshake / 120s idle timeout, you could
// fill all 8 SSE_MAX_CHANNELS by reloading logs.html ~6 times in 15s.
// uuidv4 is hoisted (function declaration further down), so the IIFE
// can call it from this earlier line.
const clientUUID = (function () {
    try {
        const KEY = 'ratgdo-logs-uuid';
        let id = localStorage.getItem(KEY);
        if (!id) {
            id = uuidv4();
            localStorage.setItem(KEY, id);
        }
        return id;
    } catch (e) {
        // Storage blocked (private browsing on iOS Safari, restrictive
        // tracking-prevention modes). Fall back to a per-session UUID —
        // not ideal, but the firmware sweep still catches the leak.
        return uuidv4();
    }
})();
// v27: best-effort cleanup. Browser releases the SSE slot on page-unload
// without waiting for the firmware orphan sweep timeout. sendBeacon does
// not block navigation and survives backgrounding on most browsers; on
// the ones it doesn't (mobile Safari background tabs), the firmware
// sweep is the safety net. No CSRF token: sendBeacon can't set custom
// headers, and the endpoint is intentionally unauthenticated (worst
// case is closing your own session early).
window.addEventListener('beforeunload', () => {
    try {
        navigator.sendBeacon('rest/events/unsubscribe?id=' + clientUUID);
    } catch (e) {
        // sendBeacon unsupported — orphan sweep will catch it
    }
});
var sysLogLoaded = false;
var tmpLogMsgs = [];

function msToTime(duration) {
    let seconds = Math.floor((duration / 1000) % 60),
        minutes = Math.floor((duration / (1000 * 60)) % 60),
        hours = Math.floor((duration / (1000 * 60 * 60)) % 24),
        days = Math.floor((duration / (1000 * 60 * 60 * 24)));

    hours = (hours < 10) ? "0" + hours : hours;
    minutes = (minutes < 10) ? "0" + minutes : minutes;
    seconds = (seconds < 10) ? "0" + seconds : seconds;

    return days + " days " + hours + " hrs " + minutes + " mins " + seconds + " secs";
}

function openTab(evt, tabName) {
    var i, tabcontent, tablinks;
    // Get all elements with class="tabcontent" and hide them
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }
    document.getElementById("clearLogBtn").style.display = "none";
    document.getElementById("reloadLogButton").style.display = "none";
    document.getElementById("clearBtn").style.display = "none";
    // Get all elements with class="tablinks" and remove the class "active"
    tablinks = document.getElementsByClassName("tablinks");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    // Show the current tab, and add an "active" class to the button that opened the tab
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
    if (tabName === "logTab") {
        document.getElementById("clearLogBtn").style.display = "inline-block";
        document.getElementById("reloadLogButton").style.display = "inline-block";
    } else if (tabName === "crashTab") {
        if (msgJson?.crashCount != 0) {
            document.getElementById("clearBtn").style.display = "inline-block";
        }
    } else if (tabName === "statusTab") {
        // Refresh status from the server
        loaderElem.style.visibility = "visible";
        document.getElementById("statusjson").innerText = "";
        fetch("status.json")
            .then((response) => {
                if (!response.ok || response.status !== 200) {
                    reject(`Error requsting status.json, RC: ${response.status}`);
                } else {
                    return response.text();
                }
            })
            .then((text) => {
                msgJson = JSON.parse(text);
                document.getElementById("statusjson").innerText = text;
                loaderElem.style.visibility = "hidden";
            })
            .catch(error => console.warn(error));
    }
}

async function loadLogs() {
    sysLogLoaded = false;
    tmpLogMsgs.length = 0;
    // Load all the logs in parallel, showing progress indicator while we do...
    loaderElem.style.visibility = "visible";
    console.log("Subscribe to Server Sent Events");
    // v27: heartbeat=10 (was 0). The orphan sweep on the firmware needs
    // a Ticker driving SSEheartbeat for class-5b cleanup (TCP-dropped
    // sockets that haven't seen a broadcast yet). Firmware coerces
    // heartbeat=0 → 30 anyway so this is mostly cache-safety; the
    // request also keeps lastActivity fresh, preventing class-5c idle reaps.
    fetch("rest/events/subscribe?id=" + clientUUID + "&log=1&heartbeat=10")
        .then((response) => {
            if (!response.ok || response.status !== 200) {
                reject(`Error registering for Server Sent Events, RC: ${response.status}`);
            } else {
                return response.text();
            }
        })
        .then((text) => {
            const evtUrl = text + '?id=' + clientUUID;
            console.log(`Register for Server Sent Events at ${evtUrl}`);
            evtSource = new EventSource(evtUrl);
            evtSource.onopen = () => {
                console.log("Load each log page");
                loadLogPages();
            };
            evtSource.addEventListener("logger", (event) => {
                // v22: HomeKit lines are EXCLUSIVE to the HomeKit tab —
                // they no longer also clutter the system log. If a line
                // matches isHomeKitLine() it goes ONLY to the HomeKit
                // pre, otherwise it goes ONLY to the system log pre.
                if (isHomeKitLine(event.data)) {
                    let hkPane = document.getElementById("homekitTab");
                    let hkScroll = (hkPane.scrollHeight - hkPane.scrollTop - hkPane.clientHeight) < 10;
                    document.getElementById("homekitlog").insertAdjacentText('beforeend', event.data + "\n");
                    if (hkScroll) hkPane.scrollTop = hkPane.scrollHeight;
                } else {
                    let divElem = document.getElementById("logTab");
                    let scroll = (divElem.scrollHeight - divElem.scrollTop - divElem.clientHeight) < 10;
                    document.getElementById("showlog").insertAdjacentText('beforeend', event.data + "\n");
                    if (!sysLogLoaded) tmpLogMsgs.push(event.data);
                    if (scroll) divElem.scrollTop = divElem.scrollHeight;
                }
            });
            evtSource.addEventListener("error", (event) => {
                // If an error occurs close the connection.
                console.log(`SSE error occurred while attempting to connect to ${evtSource.url}`);
                evtSource.close();
            });
        })
        .catch((error) => {
            console.warn(`Failed to register for Server Sent Events: ${error}`);
        });
}

async function loadLogPages() {
    // Load the pages in background
    Promise.allSettled([

        fetch("showlog")
            .then((response) => {
                if (!response.ok || response.status !== 200) {
                    reject(`Error requesting logs, RC: ${response.status}`);
                } else {
                    return response.text();
                }
            })
            .then((text) => {
                sysLogLoaded = true;
                // reduce newlines down to single \n
                text = text.replaceAll('\r\n', '\n');
                while (line = tmpLogMsgs.pop()) {
                    console.log(`Remove dup: ${line}`);
                    text = text.replace(line + '\n', '');
                }
                // v22: split the buffered showlog into HomeKit lines vs
                // system lines and seed each tab with ONLY its own
                // content. Mirror behaviour of the live SSE handler so
                // homekit lines never appear in the system log.
                const lines = text.split('\n');
                const hkLines = lines.filter(isHomeKitLine);
                const sysLines = lines.filter(l => !isHomeKitLine(l));
                document.getElementById("showlog").insertAdjacentText('afterbegin', sysLines.join('\n'));
                let divElem = document.getElementById("logTab");
                divElem.scrollTop = divElem.scrollHeight;
                if (hkLines.length > 0) {
                    document.getElementById("homekitlog").insertAdjacentText('afterbegin', hkLines.join('\n') + '\n');
                    let hkPane = document.getElementById("homekitTab");
                    hkPane.scrollTop = hkPane.scrollHeight;
                }
            })
            .catch(error => console.warn(error)),

        fetch("status.json")
            .then((response) => {
                if (!response.ok || response.status !== 200) {
                    reject(`Error requesting status.json, RC: ${response.status}`);
                } else {
                    return response.text();
                }
            })
            .then((text) => {
                msgJson = JSON.parse(text);
                document.getElementById("deviceName").innerHTML = msgJson.deviceName;
                document.title = msgJson.deviceName;
                document.getElementById("statusjson").innerText = text;
            })
            .catch(error => console.warn(error)),

        fetch("showrebootlog")
            .then((response) => {
                if (!response.ok || response.status !== 200) {
                    reject(`Error requesting reboot logs, RC: ${response.status}`);
                } else {
                    return response.text();
                }
            })
            .then((text) => {
                document.getElementById("rebootlog").innerText = text;
            })
            .catch(error => console.warn(error)),

        fetch("crashlog")
            .then((response) => {
                if (!response.ok || response.status !== 200) {
                    reject(`Error requesting crash logs, RC: ${response.status}`);
                } else {
                    return response.text();
                }
            })
            .then((text) => {
                document.getElementById("crashlog").innerText = text;
            })
            .catch(error => console.warn(error)),
    ])
        .then((results) => {
            // Once all loaded reset the progress indicator
            loaderElem.style.visibility = "hidden";
            console.log("All logs loaded");
            //console.log(results);
        });
}

async function clearLog(reload) {
    // Erase current content
    document.getElementById("showlog").innerText = "";
    document.getElementById("showLogHeader").innerHTML = "";
    // Load logs
    if (reload) loadLogs();
}

async function clearCrashLog() {
    loaderElem.style.visibility = "visible";
    await fetch('clearcrashlog');
    document.getElementById("clearBtn").style.display = "none";
    if (msgJson) msgJson.crashCount = 0;
    document.getElementById("crashlog").innerText = "No crashes saved";
    loaderElem.style.visibility = "hidden";
}

// Match HomeKit / WiFi / HomeSpan-related lines from the system log
// stream. Used by both the live SSE subscription and the initial fetch
// of /showlog when the HomeKit tab opens.
function isHomeKitLine(line) {
    return /ratgdo-homekit|HomeKit |HomeSpan|WiFi |Wifi |wifi |force-close to clear|HomeKit reconnect/i.test(line);
}

async function reconnectHomeKit() {
    const status = document.getElementById("reconnectStatus");
    if (!confirm("Reconnect HomeKit? This cycles WiFi briefly (no reboot).")) return;
    status.textContent = "Sending…";
    try {
        const res = await fetch("reconnectHomeKit", { method: "POST" });
        if (res.ok) {
            status.textContent = "OK — WiFi cycling. Watch the log below.";
            status.style.color = "#3a7a3a";
        } else {
            status.textContent = `Failed (HTTP ${res.status})`;
            status.style.color = "#a33";
        }
    } catch (e) {
        // The cycle will briefly drop our HTTP — the request may error;
        // that's expected. Don't treat a network drop as failure.
        status.textContent = "Sent (HTTP dropped during reconnect — expected).";
        status.style.color = "#3a7a3a";
    }
    setTimeout(() => { status.textContent = ""; status.style.color = ""; }, 8000);
}

async function refreshHomeKitMDNS() {
    const status = document.getElementById("reconnectStatus");
    status.textContent = "Sending…";
    try {
        const res = await fetch("refreshHomeKitMDNS", { method: "POST" });
        if (res.ok) {
            status.textContent = "OK — mDNS re-advertised. Check Home app in a few seconds.";
            status.style.color = "#3a7a3a";
        } else {
            status.textContent = `Failed (HTTP ${res.status})`;
            status.style.color = "#a33";
        }
    } catch (e) {
        status.textContent = `Failed (${e.message || "network error"})`;
        status.style.color = "#a33";
    }
    setTimeout(() => { status.textContent = ""; status.style.color = ""; }, 8000);
}

async function dumpHomeKitState() {
    const status = document.getElementById("reconnectStatus");
    status.textContent = "Sending…";
    try {
        const res = await fetch("dumpHomeKitState", { method: "POST" });
        if (res.ok) {
            status.textContent = "OK — state dump in the log below (HomeSpan output is multi-line).";
            status.style.color = "#3a7a3a";
        } else {
            status.textContent = `Failed (HTTP ${res.status})`;
            status.style.color = "#a33";
        }
    } catch (e) {
        status.textContent = `Failed (${e.message || "network error"})`;
        status.style.color = "#a33";
    }
    setTimeout(() => { status.textContent = ""; status.style.color = ""; }, 8000);
}
// Generate a UUID.  Cannot use crypto.randomUUID() because that will only run
// in a secure environment, which is not possible with ratgdo.
function uuidv4() {
    return "10000000-1000-4000-8000-100000000000".replace(/[018]/g, c =>
        (+c ^ crypto.getRandomValues(new Uint8Array(1))[0] & 15 >> +c / 4).toString(16)
    );
}
