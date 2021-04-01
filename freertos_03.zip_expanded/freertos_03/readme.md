## Programm

Es laufen 4 Tasks gleichzeitig

Die ersten drei allocieren heap speicher und rufen eine rekursive Funktion auf die immer mehr stack braucht weil sie sich immer öfter selbst auf ruft.

Der 4. Task gibt den Übrigen Heap aus und die HighWaterMark der andern drei Tasks.

## Ausführung

![chrash](crash.png)

Wie zu sehen ist wird der freie speicher (heap left) immer kleiner und als er dann 0 ereicht hängt sich das Programm auf und bleibt stehen.