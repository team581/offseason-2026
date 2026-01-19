package com.team581.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class ReusableOptional<T> {
  protected T value;
  protected boolean isPresent;

  protected ReusableOptional(T value) {
    this.value = value;
    this.isPresent = false;
  }

  public void ifPresent(Consumer<T> consumer) {
    if (isPresent) {
      consumer.accept(value);
    }
  }

  public boolean isEmpty() {
    return !isPresent;
  }

  public boolean isPresent() {
    return isPresent;
  }

  public ReusableOptional<T> or(Supplier<ReusableOptional<T>> other) {
    if (isPresent) {
      return this;
    }

    return other.get();
  }

  public T orElse(T other) {
    if (isPresent) {
      return value;
    }

    return other;
  }

  public T orElseThrow() {
    return value;
  }
}
