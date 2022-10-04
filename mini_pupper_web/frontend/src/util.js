export function formatTime(d, timeFormat) {
  d = Number(d)
  const h = Math.floor(d / 3600)
  const m = Math.floor((d % 3600) / 60)
  const s = Math.floor((d % 3600) % 60)

  const hoursDisplay = timeFormat.hours ? (h < 10 ? '0' + h : h) : ''
  const minutesDisplay = timeFormat.minutes ? (m < 10 ? '0' + m : m) : ''
  const secondsDisplay = timeFormat.seconds ? (s < 10 ? '0' + s : s) : ''

  return [hoursDisplay, minutesDisplay, secondsDisplay].reduce(
    (acc, curr, currentIndex) => (curr ? acc.concat(currentIndex === 0 ? curr : `:${curr}`) : acc),
    ''
  )
}
