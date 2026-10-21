export interface Handler {
  update(): void;
  dispose?(): void;
}
