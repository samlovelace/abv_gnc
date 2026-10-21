import * as ROSLIB from "roslib";

export class RosClient {
  #ros: ROSLIB.Ros;
  #topics = new Map<string, ROSLIB.Topic>();
  #latest = new Map<string, unknown>();
  #services = new Map<string, ROSLIB.Service>();

  constructor(url: string) {
    this.#ros = new ROSLIB.Ros({ url });
  }

  watch<T>(topic: string, messageType: string, onMsg?: (msg: T) => void): void {
    const t = new ROSLIB.Topic({ ros: this.#ros, name: topic, messageType });
    t.subscribe((msg) => {
      this.#latest.set(topic, msg);
      onMsg?.(msg as T);
    });
    this.#topics.set(topic, t);
  }

  unwatch(topic: string): void {
    this.#topics.get(topic)?.unsubscribe();
    this.#topics.delete(topic);
  }

  latest<T>(topic: string): T | undefined {
    if (!this.#topics.has(topic))
      console.warn(`latest(): '${topic}' was never watched`);
    return this.#latest.get(topic) as T | undefined;
  }

  #getService(name: string, serviceType: string): ROSLIB.Service {
    let s = this.#services.get(name);

    if (!s) {
      s = new ROSLIB.Service({ ros: this.#ros, name, serviceType });
      this.#services.set(name, s);
    }

    return s;
  }

  callService<Req extends object, Res = unknown>(
    name: string,
    serviceType: string,
    request: Req,
    timeoutMs = 5000,
  ): Promise<Res> {
    const service = this.#getService(name, serviceType);

    return new Promise<Res>((resolve, reject) => {
      const timer = setTimeout(
        () =>
          reject(
            new Error(`service '${name}' timed out after ${timeoutMs} ms`),
          ),
        timeoutMs,
      );

      service.callService(
        request,
        (res) => {
          clearTimeout(timer);
          resolve(res as Res);
        },
        (err) => {
          clearTimeout(timer);
          reject(new Error(`service '${name}' failed: ${err}`));
        },
      );
    });
  }
}
