/* ── HoloAssist — Shared JS ── */
const _GH_SVG = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M9 19c-5 1.5-5-2.5-7-3m14 6v-3.87a3.37 3.37 0 0 0-.94-2.61c3.14-.35 6.44-1.54 6.44-7A5.44 5.44 0 0 0 20 4.77 5.07 5.07 0 0 0 19.91 1S18.73.65 16 2.48a13.38 13.38 0 0 0-7 0C6.27.65 5.09 1 5.09 1A5.07 5.07 0 0 0 5 4.77a5.44 5.44 0 0 0-1.5 3.78c0 5.42 3.3 6.61 6.44 7A3.37 3.37 0 0 0 9 18.13V22"/></svg>';

function buildDock(items, opts) {
    opts = opts || {};
    var BASE = opts.base || 64, MAG = opts.mag || 88, DIST = opts.dist || 160;

    var wrap = document.createElement('aside');
    wrap.className = 'dock-wrap';
    var nav = document.createElement('nav');
    nav.className = 'dock';
    nav.setAttribute('aria-label', 'Page navigation');

    var links = items.map(function(item) {
        var a = document.createElement('a');
        a.className = 'di';
        a.href = item.href;
        if (item.id) a.dataset.id = item.id;
        a.setAttribute('aria-label', item.label);
        a.innerHTML = '<span class="di-icon">' + item.svg + '</span><span class="di-label">' + item.label + '</span>';
        nav.appendChild(a);
        return a;
    });

    nav.appendChild(Object.assign(document.createElement('div'), { className: 'd-sep' }));

    var ghLink = document.createElement('a');
    ghLink.className = 'di gh';
    ghLink.href = 'https://github.com/John-A-Chen/HoloAssist';
    ghLink.target = '_blank'; ghLink.rel = 'noreferrer';
    ghLink.setAttribute('aria-label', 'GitHub');
    ghLink.innerHTML = '<span class="di-icon">' + _GH_SVG + '</span><span class="di-label">GitHub</span>';
    nav.appendChild(ghLink);

    wrap.appendChild(nav);
    document.body.appendChild(wrap);

    var sectionLinks = links.filter(function(l) { return l.dataset.id; });
    if (sectionLinks.length) {
        var io = new IntersectionObserver(function(es) {
            es.forEach(function(e) {
                if (e.isIntersecting)
                    sectionLinks.forEach(function(l) { l.classList.toggle('active', l.dataset.id === e.target.id); });
            });
        }, { threshold: 0.35 });
        document.querySelectorAll('section[id]').forEach(function(s) { io.observe(s); });
    }

    function setW(el, n) { el.style.setProperty('--sz', n + 'px'); }
    function baseSize() { return window.innerWidth <= 768 ? Math.min(BASE, 34) : BASE; }
    function reset() { [].slice.call(nav.querySelectorAll('.di')).forEach(function(l) { setW(l, baseSize()); }); }

    nav.addEventListener('pointermove', function(e) {
        if (window.innerWidth <= 768) return;
        [].slice.call(nav.querySelectorAll('.di')).forEach(function(l) {
            var r = l.getBoundingClientRect(), c = r.top + r.height / 2;
            var inf = Math.max(0, 1 - Math.abs(e.clientY - c) / DIST);
            setW(l, BASE + (MAG - BASE) * inf * inf * (3 - 2 * inf));
        });
    });
    nav.addEventListener('pointerleave', reset);
    window.addEventListener('resize', reset);
    reset();
}
