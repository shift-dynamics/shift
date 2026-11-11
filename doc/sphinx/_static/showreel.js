/* Plays the videos of the about page in turn on the front page. Two stacked
 * video elements take turns, so that the next one loads while the current one
 * plays. A viewer who prefers reduced motion gets the first video, stopped,
 * with its controls. */
(function () {
  var clips = [
    {src: "_static/raptor.webm",
     caption: "Ford F-150 Raptor R on rolling ground, 5x faster than real " +
              "time (model by VTX, CC-BY-NC-SA-4.0)"},
    {src: "_static/klann_walker.webm",
     caption: "A walker on eight Klann legs, driven by one controller"},
    {src: "_static/bricard.webm",
     caption: "Bricard's mechanism, a paradoxical linkage"},
    {src: "_static/track_frame.webm",
     caption: "A track frame in contact on the exact surfaces of its STEP " +
              "parts"},
    {src: "_static/m1a2.webm",
     caption: "An M1A2 tank on two tracks of 78 links (model by " +
              "coderslava402, CC-BY-4.0)"}
  ];

  var figure = document.querySelector(".showreel");
  var videos = figure.querySelectorAll("video");
  var caption = figure.querySelector("figcaption");
  var current = 0;
  var index = 0;

  function Show(i) {
    caption.textContent = clips[i].caption;
  }

  if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) {
    videos[0].removeAttribute("autoplay");
    videos[0].pause();
    videos[0].controls = true;
    videos[1].remove();
    figure.classList.add("still");
    Show(0);
    return;
  }

  function LoadNext() {
    videos[1 - current].src = clips[(index + 1) % clips.length].src;
    videos[1 - current].load();
  }

  function Advance() {
    var next = videos[1 - current];
    index = (index + 1) % clips.length;
    next.currentTime = 0;
    next.play();
    next.classList.add("playing");
    videos[current].classList.remove("playing");
    current = 1 - current;
    Show(index);
    LoadNext();
  }

  videos.forEach(function (video) {
    video.addEventListener("ended", function () {
      if (video === videos[current]) {
        Advance();
      }
    });
    video.addEventListener("error", function () {
      if (video === videos[current]) {
        Advance();
      }
    });
  });

  /* the attribute would also start this element on each clip it preloads */
  videos[0].removeAttribute("autoplay");
  videos[0].play();
  videos[0].classList.add("playing");
  Show(0);
  LoadNext();
})();
