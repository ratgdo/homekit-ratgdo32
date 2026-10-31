const assert = require('node:assert/strict');
const { test } = require('node:test');
const { readFileSync } = require('node:fs');
const vm = require('node:vm');

function page({ subscriptionFails = false } = {}) {
    const timers = [], sources = [], requests = [], elements = new Map();
    const element = id => {
        if (!elements.has(id)) elements.set(id, { style: {}, innerText: '', scrollHeight: 0,
            scrollTop: 0, clientHeight: 0, insertAdjacentText(where, text) {
                this.innerText = where === 'afterbegin' ? text + this.innerText : this.innerText + text;
            } });
        return elements.get(id);
    };
    const context = vm.createContext({
        console: { log() {}, warn() {}, error() {} },
        crypto: require('node:crypto').webcrypto, Uint8Array, TextEncoder,
        document: { getElementById: element }, loaderElem: element('loader'),
        setTimeout(fn) { timers.push(fn); return timers.length; },
        clearTimeout(id) { timers[id - 1] = undefined; },
        fetch: async url => {
            requests.push(url);
            if (url.startsWith('rest/events/subscribe') && subscriptionFails) {
                subscriptionFails = false;
                throw new Error('offline');
            }
            const text = url === 'status.json' ? '{"deviceName":"Garage"}' :
                url.startsWith('rest/events/subscribe') ? '/rest/events/0' :
                url === 'showlog' ? 'history\n' : '';
            return { ok: true, status: 200, text: async () => text };
        },
        EventSource: class {
            constructor(url) { this.url = url; this.handlers = {}; sources.push(this); }
            addEventListener(name, fn) { this.handlers[name] = fn; }
            close() { this.closed = true; }
            emit(name, data) { (name === 'open' ? this.onopen : this.handlers[name])?.({ data }); }
        },
    });
    vm.runInContext(readFileSync('src/www/logs.js', 'utf8'), context);
    // Authentication is orthogonal to transport recovery and has its own 50 ms UI timer.
    context.checkAuth = async () => true;
    return { context, sources, requests, element, timers,
        async settle() { for (let i = 0; i < 30; i++) await Promise.resolve(); },
        retry() { const index = timers.findIndex(Boolean); assert.notEqual(index, -1, 'a reconnect is scheduled');
            const fn = timers[index]; timers[index] = undefined; fn(); },
    };
}

test('first SSE connection failure retries registration and loads logs without a page reload', async () => {
    const p = page();
    await p.context.loadLogs(); await p.settle();
    p.sources[0].emit('error');
    p.retry(); await p.settle();
    assert.equal(p.requests.filter(url => url.startsWith('rest/events/subscribe')).length, 2);
    p.sources[1].emit('open'); await p.settle();
    assert.equal(p.element('showlog').innerText, 'history\n');
    assert.equal(p.element('loader').style.visibility, 'hidden');
});

test('offline subscribe request retries and preserves event delivery after reconnect', async () => {
    const p = page({ subscriptionFails: true });
    await p.context.loadLogs(); await p.settle();
    p.retry(); await p.settle();
    p.sources[0].emit('open'); await p.settle();
    p.sources[0].emit('logger', 'live');
    p.sources[0].emit('error'); p.retry(); await p.settle();
    p.sources[1].emit('open'); await p.settle();
    p.sources[1].emit('logger', 'reconnected');
    assert.equal(p.element('showlog').innerText, 'history\nlive\nreconnected\n');
    assert.equal(p.requests.filter(url => url === 'showlog').length, 1);
});

test('Reload cancels a pending retry', async () => {
    const p = page();
    await p.context.loadLogs(); await p.settle();
    const first = p.sources[0];
    first.emit('error');
    await p.context.clearLog(true); await p.settle();
    assert.equal(first.closed, true);
    assert.equal(p.timers.filter(Boolean).length, 0);
    assert.equal(p.sources.length, 2);
});

test('temporary HTTP subscription rejection retries before loading logs', async () => {
    const p = page();
    const fetch = p.context.fetch;
    let rejectOnce = true;
    p.context.fetch = async url => {
        if (url.startsWith('rest/events/subscribe') && rejectOnce) {
            rejectOnce = false;
            return { ok: false, status: 503 };
        }
        return fetch(url);
    };
    await p.context.loadLogs(); await p.settle();
    assert.equal(p.sources.length, 0);
    p.retry(); await p.settle();
    p.sources[0].emit('open'); await p.settle();
    assert.equal(p.element('showlog').innerText, 'history\n');
});

test('Reload closes an established stream before opening its replacement', async () => {
    const p = page();
    await p.context.loadLogs(); await p.settle();
    const first = p.sources[0];
    first.emit('open'); await p.settle();
    await p.context.clearLog(true); await p.settle();
    assert.equal(first.closed, true);
    p.sources[1].emit('open'); await p.settle();
    assert.equal(p.element('showlog').innerText, 'history\n');
});

test('Reload ignores a subscription response from the previous load', async () => {
    const p = page();
    const fetch = p.context.fetch;
    let resolveSubscription;
    p.context.fetch = url => url.startsWith('rest/events/subscribe') ?
        new Promise(resolve => { resolveSubscription = resolve; }) : fetch(url);
    await p.context.loadLogs(); await p.settle();
    p.context.fetch = fetch;
    await p.context.clearLog(true); await p.settle();
    resolveSubscription({ ok: true, status: 200, text: async () => '/rest/events/old' });
    await p.settle();
    assert.equal(p.sources.length, 1);
    assert.equal(p.sources[0].url.startsWith('/rest/events/0?'), true);
});
