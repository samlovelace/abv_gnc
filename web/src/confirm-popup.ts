// Small reusable confirm popup: shows a one-line message and a single
// confirm button near a given screen position, dismissed by confirming,
// clicking elsewhere, or pressing Escape. Used by TableInteractionHandler's
// three confirm flows (send goal / add obstacle / clear obstacles) instead
// of window.confirm(), which blocks the requestAnimationFrame loop while
// open and can't be themed to match the rest of the app.
//
// Dismissing WITHOUT confirming (outside click / Escape / a new popup
// superseding this one) fires the optional onCancel - abv_gui's own
// equivalent (a QMenu with a single action) treats "dismissed without
// choosing the action" as a cancel too (e.g. clearing the goal ghost).

let current: HTMLDivElement | null = null;
let currentOnCancel: (() => void) | null = null;

function teardown(): void {
  current?.remove();
  current = null;
  document.removeEventListener("pointerdown", onOutsideClick, true);
  document.removeEventListener("keydown", onKeyDown, true);
}

function cancel(): void {
  const onCancel = currentOnCancel;
  teardown();
  currentOnCancel = null;
  onCancel?.();
}

function onOutsideClick(e: PointerEvent): void {
  if (current && !current.contains(e.target as Node)) cancel();
}

function onKeyDown(e: KeyboardEvent): void {
  if (e.key === "Escape") cancel();
}

export function showConfirmPopup(
  message: string,
  clientX: number,
  clientY: number,
  confirmLabel: string,
  onConfirm: () => void,
  onCancel?: () => void,
): void {
  // supersede any still-open popup without treating it as a user cancel
  teardown();
  currentOnCancel = null;

  const popup = document.createElement("div");
  popup.className = "confirm-popup";
  popup.style.left = `${clientX}px`;
  popup.style.top = `${clientY}px`;

  const text = document.createElement("div");
  text.className = "confirm-popup-message";
  text.textContent = message;
  popup.appendChild(text);

  const button = document.createElement("button");
  button.className = "confirm-popup-button";
  button.textContent = confirmLabel;
  button.addEventListener("click", (e) => {
    e.stopPropagation();
    currentOnCancel = null; // confirmed, not cancelled
    teardown();
    onConfirm();
  });
  popup.appendChild(button);

  document.body.appendChild(popup);
  current = popup;
  currentOnCancel = onCancel ?? null;

  // deferred so the same mouseup/click that triggered this doesn't
  // immediately dismiss it via the outside-click listener
  setTimeout(() => {
    document.addEventListener("pointerdown", onOutsideClick, true);
    document.addEventListener("keydown", onKeyDown, true);
  }, 0);
}
